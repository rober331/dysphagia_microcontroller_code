/*
 * Summer Scholars project for force sensor integration.
 *
 * Original Author:  Ruben Roberts
 * Date:    6/7/2023
 * 
 * Adjusted: Julian Quevedo
 * Date:    1/29/2024
 */

#include <BluetoothSerial.h>

#define SENSOR1 36            // FSR 1 and 10K pulldown are connected to IO36
#define SENSOR2 37            // FSR 2 and 10K pulldown are connected to IO37
#define SENSOR3 38            // FSR 3 and 10K pulldown are connected to IO38
#define SENSOR4 39            // FSR 4 and 10K pulldown are connected to IO39
#define SENSOR5 32            // FSR 5 and 10K pulldown are connected to IO32
#define SENSOR6 33            // FSR 6 and 10K pulldown are connected to IO33
#define SENSOR7 34            // FSR 7 and 10K pulldown are connected to IO34

#define USR_BTN 0             // Used to read the state of the user button
#define EXP_SCALER  (1/0.69)  // Exponent for conversion from measured FSR resistance to grams

//
const int ledPin = 4;           // GPIO 4 corresponds to IO4 on ESP32
//

const float referenceVoltage = 3330.0;  // Reference voltage for the ADC
const float resistorValue = 10000.0;    // Resistance value of the 10k resistor
const float maxADCValue = 4095.0;       // Maximum ADC value (12-bit resolution)

const float minForce = 10.0;    // Minimum force in grams
const float maxForce = 5000.0;  // Maximum force in grams
const float gravity = 9.8;      // Acceleration due to gravity in m/s^2

int forceSensorValues[7];       // Array to store force sensor readings
bool readingSensors = true;
float forcekPaValues[7];     // The resulting newtons for each FSR

BluetoothSerial SerialBT;

void printValue(String header, int value) {
  Serial.print(header);
  Serial.print(": ");
  Serial.print(value);
}

float getForceInkPa(int sensorValue, bool lastSensor) {
  float voltage = map(sensorValue, 0, maxADCValue, 0, referenceVoltage); // Map ADC value to voltage range
  float fsr_resistance = resistorValue * ((referenceVoltage/voltage) - 1);
  fsr_resistance = lastSensor ? fsr_resistance * 10 : fsr_resistance;
  fsr_resistance = (fsr_resistance > 10000000) ? 10000000 : fsr_resistance;
  fsr_resistance = (fsr_resistance < 0.005) ? 0.005 : fsr_resistance;
  float force_grams = -log(fsr_resistance/2000000) / 0.0006;
  force_grams = (force_grams < 0.0) ? 0.0 : force_grams;
  //float force_grams = map(voltage, 0, referenceVoltage, minForce, maxForce); // Map voltage to force range

  float force_kpa = force_grams * gravity / 1000; // Convert force from grams to kPa
  return (force_kpa < 1.0) ? 0.0 : force_kpa;
}

void displayToSerialMonitor() {
  // Print the force sensor values via serial communication
  for (int i = 0; i < 7; i++) {
    /*Serial.print("SENSOR ");
    Serial.print(i+1);
    Serial.print(": ADC Value: ");
    Serial.print(forceSensor);
    Serial.print(", Voltage: ");
    Serial.print(voltage/1000, 3); // Print voltage with 3 decimal places
    Serial.print("V, Force: ");
    Serial.print(force_newtons, 2); // Print force in Newtons with 2 decimal places
    Serial.println(" N");*/

    if(i < 6) {
      Serial.print(forcekPaValues[i], 2);
      Serial.print(",");
    } else {
      Serial.println(forcekPaValues[i], 2);
    }
  }
}

void performBluetoothCommunications() {
  while (readingSensors) {
    forcekPaValues[0] = getForceInkPa(analogRead(SENSOR1), false);
    forcekPaValues[1] = getForceInkPa(analogRead(SENSOR2), false);
    forcekPaValues[2] = getForceInkPa(analogRead(SENSOR3), false);
    forcekPaValues[3] = getForceInkPa(analogRead(SENSOR4), false);
    forcekPaValues[4] = getForceInkPa(analogRead(SENSOR5), false);
    forcekPaValues[5] = getForceInkPa(analogRead(SENSOR6), false);
    forcekPaValues[6] = getForceInkPa(analogRead(SENSOR7), true);

    displayToSerialMonitor();

    // Send force values over Bluetooth
    SerialBT.write(reinterpret_cast<uint8_t*>(&forcekPaValues), sizeof(forcekPaValues));
    /*SerialBT.print(forcekPaValues[0], 2);
    SerialBT.print(",");
    SerialBT.print(forcekPaValues[1], 2);
    SerialBT.print(",");
    SerialBT.print(forcekPaValues[2], 2);
    SerialBT.print(",");
    SerialBT.println(forcekPaValues[3], 2);*/

    Serial.print("Reading USR_BTN: ");
    Serial.println(analogRead(USR_BTN));
    Serial.println("");
    if(analogRead(USR_BTN) == 0) {
      Serial.println("Button pressed. Terminating reading sensors.");
      readingSensors = false;
    }

    delay(1000);
  }
}

void setup() {
  pinMode(ledPin,OUTPUT); // Set the LED pin as an output
  digitalWrite(ledPin, HIGH);
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT"); // Bluetooth device name
  Serial.println("Classic Bluetooth device active, waiting for connections...");
}

void loop() {
  if (readingSensors) {
    performBluetoothCommunications();
    //displayToSerialMonitor();
  }
}
