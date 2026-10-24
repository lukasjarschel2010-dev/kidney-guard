/*
 * FSR Voltage Divider Reading for XIAO ESP32-S3
 * Pin D8 (Analog A8, GPIO7) with a 10k Pull-down Resistor to GND
 */

// On the XIAO ESP32-S3, pin D8 (GPIO7) is mapped to analog channel A8
const int fsrPin = A8; 

void setup() {
  // Initialize serial communication at 115200 baud for ESP32
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for the serial port to connect (needed for native USB)
  }
  
  // Configure the FSR pin as an input
  pinMode(fsrPin, INPUT);
  
  Serial.println("XIAO ESP32-S3 FSR Measurement Initialized.");
}

const int sampleIntervalMs = 10;   // 100 samples per second
const int printIntervalMs = 100;   // 10 prints per second

int peakAdcValue = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPrintTime = 0;

void loop() {
  unsigned long now = millis();

  // 1. Sample at 100 Hz, keeping the highest ADC value seen since the last print
  if (now - lastSampleTime >= sampleIntervalMs) {
    lastSampleTime = now;

    // ESP32-S3 defaults to a 12-bit ADC resolution (0 to 4095)
    int adcValue = analogRead(fsrPin);
    if (adcValue > peakAdcValue) {
      peakAdcValue = adcValue;
    }
  }

  // 2. Print only the peak value, at 10 Hz
  if (now - lastPrintTime >= printIntervalMs) {
    lastPrintTime = now;

    int adcValue = peakAdcValue;
    peakAdcValue = 0; // reset for the next window

    // Convert the raw reading to Voltage (ESP32-S3 logic is 3.3V)
    float voltage = (adcValue * 3.3) / 4095.0;

    // Print the raw data to the Serial Monitor / Serial Plotter
    Serial.print("ADC Raw: ");
    Serial.print(adcValue);
    Serial.print("\t Voltage: ");
    Serial.print(voltage, 2);
    Serial.print(" V\t");

    // Provide a qualitative description of the force applied
    if (adcValue < 100) {
      Serial.println("[ No Pressure ]");
    } else if (adcValue < 1200) {
      Serial.println("[ Light Touch ]");
    } else if (adcValue < 2800) {
      Serial.println("[ Medium Squeeze ]");
    } else {
      Serial.println("[ Heavy Squeeze ]");
    }
  }
}
