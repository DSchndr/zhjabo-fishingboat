#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <PS4Controller.h>
//#include "esp_gap_bt_api.h"

// GPIO CONFIG

/* 25: LED Grün hinten
** 26: LED Rot hinten
** 12: Motor ?
** 13: Motor ?
** 22: LED Weiss unten
** 23: ??? goes to charging board (white wire) but goes... nowhere? timer / port on mcu
** 27: LED Weiss vorne
** 35: Battery voltage / 50k + 50k
** 33: Hebebühne
** 34: ??? Goes to epoxied 2 pin part which has ~10kohm resistance, goes to adc / port on mcu probably temp sensor
** D34: echo
** D32: trig
*/

// leds on the boat
#define LED_GREEN 25
#define LED_RED 26 
#define LED_UNTEN 22
#define LED_VORNE 14
#define BUEHNE 33
// brushed esc
#define MOTOR_L 13
#define MOTOR_R 12
// adcs for temperature (to correct distance) and battery (to know when its empty)
#define BAT_ADC 35
//50k - 50k voltage divider from vbat
#define TEMP_ADC 39
// aj-sr04m connections to get water depth / "fish finder", leave r19 open since we have a different constant in water...
#define ECHO 34
#define TRIG 32

#define VOLTAGE_MAX 4.2
#define VOLTAGE_MIN 3.3

void GPIOInit();
int measureDistance();
double getTemperature();
float getBatteryVoltage();
void initMotor();
int getBatteryPercentage();
void setMotorLeft(int degrees);
void setMotorRight(int degrees);
int mapStickToDegree(int StickPos);
void setDirection(int x, int y);
void handleController();
void buttonPressGPIO(unsigned long *timevar, int gpio);
void setBatteryLevelColor(int batteryLevel);
void detatchMotor();
void controllerLedTask(void *parameter);
void controllerLedTaskSetup();
Servo servo1;
Servo servo2;
int minUs = 1000;
int maxUs = 2000;

unsigned long previousMillis_ledred = 0;
TaskHandle_t ledtaskHandle = NULL;

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  esp_log_level_set("*", ESP_LOG_DEBUG);
  /*ESP_LOGD("MyTAG", "ESP_LOGD");
  ESP_LOGI("MyTAG", "ESP_LOGI");
  ESP_LOGW("MyTAG", "ESP_LOGW");
  ESP_LOGE("MyTAG", "ESP_LOGE");
  log_d("log_d");
  log_i("log_i");
  log_w("log_w");
  log_e("log_e");*/
  Serial.println("Hello World!");
  GPIOInit();
  initMotor();
  PS4.begin("BA:BA:CA:FF:EE:01");
}

void loop()
{
  unsigned long currentMillis = millis();
  // measureDistance();
  if (PS4.isConnected())
  {
    controllerLedTaskSetup();
    handleController();
    setDirection(PS4.LStickX(), PS4.LStickY());
  }
  else
  { // blink red led to indicate that no controller is connected.
    if (ledtaskHandle != NULL) //delete task since we dont have a controller to access
    {
      vTaskDelete(ledtaskHandle);
      ledtaskHandle = NULL;
    }
    if (currentMillis - previousMillis_ledred >= 1000)
    {
      digitalWrite(LED_RED, (digitalRead(LED_RED) == LOW) ? HIGH : LOW);
      digitalWrite(LED_GREEN, (digitalRead(LED_RED) == HIGH) ? LOW : HIGH);
      previousMillis_ledred = currentMillis;
    }
    // stop motors
    setDirection(0, 0);
  }
}
void controllerLedTaskSetup()
{
  if (ledtaskHandle == NULL)
  {
    xTaskCreate(
        controllerLedTask, // Function that should be called
        "ControllerLed",   // Name of the task (for debugging)
        3000,              // Stack size (bytes)
        NULL,              // Parameter to pass
        1,                 // Task priority
        &ledtaskHandle     // Task handle
    );
  }
}

void controllerLedTask(void *parameter)
{
  Serial.print("LEDTask is running on: ");
  Serial.println(xPortGetCoreID());
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  while (true)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS); //1s
    setBatteryLevelColor(getBatteryPercentage());
    log_i("Bat: %.2fV", getBatteryVoltage());
    log_i("Temp: %.2fC", getTemperature());
    log_i("Controller Bat: %d", PS4.Battery());
  }
}

unsigned long ledmilu, ledmilo, hebmil;
void handleController()
{
  if (PS4.LStickX())
  {
    // Serial.printf("Left Stick x at %d\n", PS4.LStickX());
  }
  if (PS4.LStickY())
  {
    // Serial.printf("Left Stick y at %d\n", PS4.LStickY());
  }
  if (PS4.Triangle())
  {
    buttonPressGPIO(&ledmilo, LED_VORNE);
  }
  if (PS4.Cross())
  {
    buttonPressGPIO(&ledmilu, LED_UNTEN);
  }
  if (PS4.Circle())
  {
    buttonPressGPIO(&hebmil, BUEHNE);
  }
}

void buttonPressGPIO(unsigned long *timevar, int gpio)
{
  if (millis() - *timevar > 10)
  {
    digitalWrite(gpio, (digitalRead(gpio) == LOW) ? HIGH : LOW);
  }
  *timevar = millis();
}

bool isMotorDetached = false;
void setDirection(int x, int y)
{
  if (x < 30 && x > -30)
    x = 0; // Deadzone
  if (y < 30 && y > -30)
    y = 0;

  if (x == 0 && y == 0 && !isMotorDetached)
  { // motor whine patch
    detatchMotor();
    isMotorDetached = true;
  }
  else if (isMotorDetached)
  {
    initMotor();
    isMotorDetached = false;
  }

  int L = y + ((x < 0) ? abs(x) : 0);
  int R = y + ((x >= 0) ? x : 0);
  setMotorLeft(mapStickToDegree(L));
  setMotorRight(mapStickToDegree(R));
}

int mapStickToDegree(int StickPos)
{ // map -128 to 128 from stick to 0-180 for motor
  return map(StickPos, -128, 128, 0, 180);
}
void setMotorLeft(int degrees)
{
  servo2.write(degrees);
}
void setMotorRight(int degrees)
{
  servo1.write(degrees);
}
void initMotor()
{
  ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50); // Standard 50hz servo
  servo2.setPeriodHertz(50);
  servo1.attach(MOTOR_R, minUs, maxUs);
  servo2.attach(MOTOR_L, minUs, maxUs);
}
void detatchMotor()
{
  servo1.detach();
  servo2.detach();
  pinMode(MOTOR_L, LOW);
  pinMode(MOTOR_R, LOW);
}
void GPIOInit()
{
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);

  digitalWrite(13, LOW);
  digitalWrite(12, LOW);

  // digitalWrite(23, HIGH);
  digitalWrite(22, LOW);
  digitalWrite(14, LOW);
  digitalWrite(33, LOW); // do not trigger now.

  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(22, OUTPUT);
  // pinMode(23, OUTPUT);
  pinMode(14, OUTPUT);

  pinMode(33, OUTPUT);
  pinMode(35, INPUT);
  pinMode(39, INPUT_PULLUP);
  pinMode(34, INPUT_PULLUP);
  pinMode(32, OUTPUT);
}

// 1:1 voltage divider

float getBatteryVoltage()
{
  return ((float)analogRead(35) / 4095) * 3.3 * 2;
}
int getBatteryPercentage()
{
  return 100 * (getBatteryVoltage() - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN);
}

void setBatteryLevelColor(int batteryLevel)
{
  // map battery level to an RGB color
  int blue = 0, red = 0, green = 0;
  if (batteryLevel > 100) //honestly, if this happens your lipo has a bad time
  {
    red = 255;
  }
  else
  {
    red = (100 - batteryLevel) * 255 / 100;
    green = batteryLevel * 255 / 100;
  }
  // set the RGB color
  PS4.setLed(red, green, blue);
  PS4.sendToController();
  delay(100);
}

double getTemperature()
{
  double R1 = 10000.0;         // voltage divider resistor value
  static double Beta = 3950.0; // Beta value
  double To = 298.15;          // Temperature in Kelvin for 25 degree Celsius
  double Ro = 10000.0;         // Resistance of Thermistor at 25 degree Celsius
  double Vout, Rt = 0;
  double T, Tc, Tf = 0;
  double adc = 0;
  double adcMax = 4095.0;
  double Vs = 3.3;
  adc = analogRead(39);
  // adc = ADC_LUT[(int)adc]; //nah, doesn't have to be this correct rn...

  Vout = adc * Vs / adcMax;
  Rt = R1 * Vout / (Vs - Vout);

  T = 1 / (1 / To + log(Rt / Ro) / Beta); // Temperature in Kelvin
  Tc = T - 273.15;                        // Celsius
  // Tf = Tc * 9 / 5 + 32;              // Fahrenheit
  return Tc;
}

//crappy aj-sr04m module always gives me 19cm, i didn't expect it to be this long...
//have to implement with custom hw
int measureDistance()
{
  // unsigned long duration_us;
  //  generate 10-microsecond pulse to TRIG pin
  digitalWrite(32, HIGH);
  delayMicroseconds(30);
  digitalWrite(32, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(34, HIGH);

  // calculate the distance
  // int distance_cm = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(duration_us / 58.8235); // distance_cm);
  Serial.println(" cm");

  delay(500);
  return 0; // distance_cm;
}