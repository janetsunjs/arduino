#define TEMP_SENSOR A0
float resistance = 0.0; // Variable to store the resistance
float voltage = 0.0; // Voltage across the thermistor
float temperature = 0.0; // Temperature in Celsius
// const float knownResistor = 5200.0; // kohm The value of the voltage divider resistor (e.g., 10kΩ)
const float knownResistor = 8.1; // kohm The value of the voltage divider resistor (e.g., 10kΩ)
#define PIN_HEATER_PWM 9
#define PIN_HEATER_PWM10 10
#define PIN_HEATER_PWM11 11
#include "InterpolationLib.h"

double currentTemperature, heaterPower;


// Moving Average Variables
const int bufferSize = 10;
int count = 0;
unsigned long previousTime = 0;
double temp[bufferSize];



void setup() {
  Serial.begin(9600); // Start the serial communication
  pinMode(PIN_HEATER_PWM, OUTPUT);
}

void loop() {
  delay(10);  // Delay for a second before the next reading

  // Read the raw analog value (0-1023)
  analogWrite(PIN_HEATER_PWM, 254);
  analogWrite(PIN_HEATER_PWM10, 127);
  analogWrite(PIN_HEATER_PWM11, 60);

  int sensorValue = analogRead(TEMP_SENSOR);
  float voltage = sensorValue * (5.0 / 1023.0);
  if (voltage == 0) voltage = 0.005; // Prevent division by zero
  // if (voltage == 0) voltage = 0.06; // 0.06 == 666kOhms
  // voltage = 1.0; // 32kOhms 152.61
    voltage = sensorValue * (5.0 / 1023.0);
  float resistance = knownResistor * (5.0 / voltage - 1.0);
  currentTemperature = calculateTemperature(resistance); // Convert resistance to temperature
    // Store values in buffer for averaging
  if(count<bufferSize){
    temp[count] = currentTemperature;
    count++;
  }
  if (millis() - previousTime >= 1000) {
    double avgTemperature = 0;
    for (int i = 0; i < count; i++) {
      avgTemperature += temp[i];
    }
    avgTemperature /= count; // Compute average temperature
    count = 0;               // Reset count for new batch
    previousTime = millis(); // Update last processed time
    currentTemperature = avgTemperature; // Use smoothed value
    // // Calculate the thermistor resistance using the voltage divider formula
    // resistance = knownResistor * (5.0/voltage - 1.0);
    // // Convert the resistance to temperature using the thermistor's characteristics
    // temperature = calculateTemperature(resistance);
    Serial.print(sensorValue);
    Serial.print(",");
    Serial.print(voltage);
    // Print the temperature and resistance values
    Serial.print(",");
    Serial.print(avgTemperature);
    Serial.print(",");
    Serial.print(resistance);
    Serial.println(" ");

    // Serial.print(sensorValue);
    // Serial.print(" bits Voltage: ");
    // Serial.print(voltage);
    // // Print the temperature and resistance values
    // Serial.print("V Temperature: ");
    // Serial.print(avgTemperature);
    // Serial.print(" °C, Resistance: ");
    // Serial.print(resistance);
    // Serial.println(" kohms");
  }
}

float calculateTemperature(float resistance) {
  return 250 - 28 * log(resistance);
}