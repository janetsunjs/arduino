#include <PID_v1.h>
#include "AVR_PWM.h"
#include "InterpolationLib.h"
#include <cppQueue.h>

#define TEMP_SENSOR A0

#define PIN_STEPPER_PULSE 10
#define STEPPER_FREQ_HZ_MIN 250.00  // can't be too small - likely smallest value - from PWM timer arduino dependent

#define PIN_HEATER_PWM 6  // DIGITAL PIN ~2 [CONNECTED TO ]

// Thermistor Values
const int size = 27;  // Number of data points
const double temperatureValues[size] = { 10, 20, 25, 30, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 360, 380, 400, 420, 440, 460, 480, 500 };
const double resistanceValues[size] = { 8100, 5200, 4500, 2830, 666, 288, 136, 70, 38.5, 22.2, 13.2, 8.07, 5.12, 3.38, 2.24, 1.555, 1.1, 0.79, 0.578, 0.434, 0.333, 0.253, 0.196, 0.157, 0.1254, 0.102, 0.0837 };

// Thermistor Voltage Divider
const float knownResistor = 8.1;        // 8.1kΩ resistor
const float targetTemperature = 200.0;  // Target temperature in °C

// PID Control Variables
double currentTemperature, heaterPower;
double setpoint = targetTemperature;
double Kp = 41.1, Ki = 2.9, Kd = 12.06;  // PID tuning values

PID myPID(&currentTemperature, &heaterPower, &setpoint, Kp, Ki, Kd, DIRECT);


// Moving Average Variables
const int bufferSize = 20;
double count = 0;
double num = 0;
unsigned long previousTime = 0;
double temp[bufferSize];
cppQueue queue(sizeof(currentTemperature), bufferSize, FIFO, false);

AVR_PWM* PWM_Instance;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_HEATER_PWM, OUTPUT);
  pinMode(PIN_STEPPER_PULSE, OUTPUT);
  PWM_Instance = new AVR_PWM(PIN_STEPPER_PULSE, STEPPER_FREQ_HZ_MIN, 25.0);
  PWM_Instance->setPWM();  

  // PID Setup
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Heater PWM range //
}

void loop() {


  // int sensorValue = analogRead(TEMP_SENSOR);

  // int sensorValue = 600; // 201.22 °C 
  // int sensorValue = 580; // 198.97 °C  
  int sensorValue = 560; // 196.75 °C    

  float voltage = sensorValue * (5.0 / 1023.0);
  if (voltage == 0) voltage = 0.005;  // Prevent division by zero
  float resistance = knownResistor * (5.0 / voltage - 1.0);
  currentTemperature = calculateTemperature(resistance);  // Convert resistance to temperature

  // Store values in buffer for averaging
  if (count < bufferSize) {
    queue.push(&currentTemperature);
    // temp[count] = currentTemperature;
    count++;
  }

  if (millis() - previousTime >= 1000) {
    // Serial.print(sensorValue);
    double avgTemperature = 0;
    while (!queue.isEmpty()) {
      queue.pop(&currentTemperature);
      avgTemperature += currentTemperature;  // avgTemperature += temp[i];
    }
    avgTemperature /= count;  // Compute average temperature
    count = 0;                // Reset count for new batch
    previousTime = millis();  // Update last processed time
    // PID Compute
    myPID.Compute();
    analogWrite(PIN_HEATER_PWM, (int)heaterPower);  // Adjust heater power

    // Check everything is properly working
    // Serial.print(avgTemperature);
    // Serial.print(",");
    // Serial.print(resistance);
    // Serial.println(",");
    // Serial.println(heaterPower);
    // Serial.println(" ");

    Serial.print(sensorValue);
    Serial.print(" bits Voltage: ");
    Serial.print(voltage);
    Serial.print("V Temperature: ");
    Serial.print(avgTemperature);
    Serial.print(" °C, Resistance: ");
    Serial.print(resistance);
    Serial.println(" kohms");
    Serial.println(heaterPower);
    Serial.println(" PWM");
  }
    if (currentTemperature >= 180.0) {
      PWM_Instance->setPWM(PIN_STEPPER_PULSE, STEPPER_FREQ_HZ_MIN, 50.0);
      //} else if currentTemperature => 205.0){
    } else {
      // PWM_Instance->disablePWM(PIN_STEPPER_PULSE); // Disable PWM on the pin
      digitalWrite(PIN_STEPPER_PULSE, LOW);  // Set the pin to LOW
      // PWM_Instance->setPWM(PIN_STEPPER_PULSE, STEPPER_FREQ_HZ_MIN, 0);  // Stop extrusion if too cold
    }  
}

// Function to Convert Resistance to Temperature
float calculateTemperature(float resistance) {
  return 250 - 28 * log(resistance);
  // return Interpolation::ConstrainedSpline(resistanceValues, temperatureValues, size, resistance, true);
}
