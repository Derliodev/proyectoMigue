#include <Blynk.h>
#include <BlynkSimpleEsp32.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>

#define BLYNK_TEMPLATE_ID "TMPL29gLwY4ZO"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "ndzqLRNVdkE0sDO8wQZAAoh7kP-gPElj"

// used pins
// sda, scl -> 21 , 22
const uint8_t tdsPin = 32;
const uint8_t turbiditySensorPin = 34;
const uint8_t oneWireBus = 5; 
const uint8_t phPin = 35;
const uint8_t greenLedPin = 16;
const uint8_t yellowLedPin = 17;
const uint8_t redLedPin = 18;
const uint8_t relay1Pin = 19; // pump
const uint8_t relay2Pin = 2; // CO2
const uint8_t relay3Pin = 4; // heat
const uint8_t relay4Pin = 23; //


// ph
float phValue;
float ph_calibration_value = 20.24 - 0.7; //21.34 - 0.7
unsigned long int ph_analog_average_value; 
int16_t ph_buffer_arr[10];
int16_t ph_temp;

// lcd
const uint8_t lcdAddr = 0x27;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(lcdAddr, 20, 4);
unsigned long lcdShowPreviousMillis = 0;  // will store last time LED was updated
const long lcdShowInterval = 1000;

// tds variables
const uint16_t tdsThreshold = 500; // EPA's maximum contamination level
const float vref = 3.3;  // analog reference voltage(Volt) of the ADC
const float adcResolution = 4096; // 12 bit ADC   
const uint16_t sampleTdsPointCounts = 20;
float tdsValue;
const float tdsLowWarningSet = 150; // ppm
const float tdsHighWarningSet = 400; // ppm
int analogBuffer[sampleTdsPointCounts];
int analogBufferTemp[sampleTdsPointCounts];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;


// turbidity variables
const uint16_t lowNTUThreshold = 20;
const uint16_t mediumNTUThreshold = 50;
float voltage = 0.0;
float ntuValue = 0.0;
int ntuSamples = 500;


// ds18b20    
OneWire oneWire(oneWireBus);
DallasTemperature ds18b20(&oneWire);
float temperatureValue = 25; // ref temp if no sensor 
const float temperatureLowerSet = 25.0;
const float temperatureUpperSet = 28.0;

void setup(){
    Serial.begin(115200);

    // pin config
    pinMode(greenLedPin, OUTPUT);
    pinMode(yellowLedPin, OUTPUT);
    pinMode(redLedPin, OUTPUT);
    pinMode(tdsPin, INPUT);
    pinMode(turbiditySensorPin, INPUT);

    // ds18b20 init
    ds18b20.begin();

    // lcd init
    lcd.init();
    lcdWelcomeMessage();
    delay(1000);
}

void loop(){
    // tds
    sampleTDS();
    readAndCheckTds();

    // temp
    checkTemperature();

    // turbidity
    readAndCheckTurbidity(); 

    // ph
    readAndCheckPH();   

    // LCD SHOW
    unsigned long currentMillis = millis();
    if (currentMillis - lcdShowPreviousMillis >= lcdShowInterval)
    {
        // Save the last time you blinked the LED
        lcdShowPreviousMillis = currentMillis;
        showParametersOnLcd();
    }
}


// ===========  Helper functions  =============
void lcdWelcomeMessage(){
    lcd.backlight();
    lcd.setCursor(5, 0);
    lcd.print("BIENVENIDO");
    lcd.setCursor(9, 1);
    lcd.print("A");
    lcd.setCursor(5, 2);
    lcd.print("MEDIDOR PH");
}

void CheckTDSLevelThreshold(){
    if (tdsValue < tdsLowWarningSet){
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
    }
    else if (tdsValue > tdsHighWarningSet){
      digitalWrite(redLedPin, HIGH);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
    }
    else{
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
    }
}

void showParametersOnLcd(){
    lcd.clear();
    // Title
    // lcd.setCursor(0, 0);
    // lcd.print("==== PARAMETROS ====");
    
    //Turbidity
    lcd.setCursor(4, 0);
    lcd.print("NTU: ");
    lcd.print(ntuValue);

    // PH
    lcd.setCursor(5, 1);
    lcd.print("PH: ");
    lcd.print(phValue);

    // temp
    lcd.setCursor(6, 2);
    lcd.print("T: ");
    lcd.print(temperatureValue);
    lcd.print(" C");

    // TDS
    lcd.setCursor(4, 3);
    lcd.print("TDS: ");
    lcd.print(tdsValue, 1);
    lcd.print(" ppm");
}

float getDS18B20TemperatureC(){
    ds18b20.requestTemperatures(); 
    float temperatureC = ds18b20.getTempCByIndex(0);
    return temperatureC;
}

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void sampleTDS() {
  static unsigned long analogSampleTimePoint = millis();
  if (millis() - analogSampleTimePoint > 40U) {
    analogSampleTimePoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(tdsPin);
    analogBufferIndex++;
    if (analogBufferIndex == sampleTdsPointCounts) {
      analogBufferIndex = 0;
    }
  }
}

void readAndCheckTds() {
  static unsigned long printTimePoint = millis();
  if (millis() - printTimePoint > 800U) {
    temperatureValue = getDS18B20TemperatureC(); // get the current Temp.

    printTimePoint = millis();
    for (copyIndex = 0; copyIndex < sampleTdsPointCounts; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }

    averageVoltage = getMedianNum(analogBufferTemp, sampleTdsPointCounts) * (float)vref / adcResolution;

    float compensationCoefficient = 1.0 + 0.02 * (temperatureValue - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    CheckTDSLevelThreshold();
  }
}

void checkTemperature(){
  // the temperature was already read before calling this function
  // temperatureValue = getDS18B20TemperatureC();

  if (temperatureValue < temperatureLowerSet)
  {
    digitalWrite(relay3Pin, LOW);
  }
  else if (temperatureValue > temperatureUpperSet)
  {
    digitalWrite(relay3Pin, HIGH);
  }
}


void readAndCheckTurbidity()
{
  voltage = 0;
  for (int i = 0; i < ntuSamples; i++)
  {
    voltage += ((float)analogRead(turbiditySensorPin) / 4095.0) * 3.3; // Mapeo de la lectura analógica para 12-bit
  }
  voltage = voltage / ntuSamples;
  voltage = roundToDecimalPlaces(voltage, 1);

  // Según la curva de la gráfica y formula del sensor
  if (voltage < 1.65)
  {
    ntuValue = 3000;
  }
  else
  {
    ntuValue = -1120.4 * (voltage * voltage) + 5742.3 * voltage - 4352.9;
  }

  CheckTurbidityThresholds();
}


float roundToDecimalPlaces(float in_value, int decimal_place) {
  float multiplier = powf(10.0f, decimal_place); 
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}


void CheckTurbidityThresholds()
{
  if (ntuValue > mediumNTUThreshold)
  {
    // Print Warning
    Serial.println("El valor de Turbidez NTU esta por encima del Limite Medio (50)");
  }
}

void readAndCheckPH()
{
  for (int i = 0; i < 10; i++)
  {
    ph_buffer_arr[i] = analogRead(phPin);
    delay(30);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (ph_buffer_arr[i] > ph_buffer_arr[j])
      {
        ph_temp = ph_buffer_arr[i];
        ph_buffer_arr[i] = ph_buffer_arr[j];
        ph_buffer_arr[j] = ph_temp;
      }
    }
  }
  ph_analog_average_value = 0;
  for (int i = 2; i < 8; i++)
  {
    ph_analog_average_value += ph_buffer_arr[i];
  }

  float phVoltage = (float)ph_analog_average_value * 3.3 / adcResolution / 6;
  // float phVoltage = (float)ph_analog_average_value * 3.3 / 4096.0 / 6;
  // Serial.print("Voltage: " + (String)phVoltage);

  // getting the PH Value
  phValue = -5.70 * phVoltage + ph_calibration_value;

  checkPHThresholds();
}

void checkPHThresholds()
{
  if (phValue < 6)
  {
    digitalWrite(relay1Pin, LOW); // pump
    digitalWrite(relay2Pin, HIGH);
  }
  else if (phValue > 7)
  {
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2Pin, LOW); // co2
  }
  else
  {
    digitalWrite(relay1Pin, HIGH); // off both
    digitalWrite(relay2Pin, HIGH);
  }
}