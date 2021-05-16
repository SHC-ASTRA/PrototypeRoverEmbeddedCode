// Imports!
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <ADC.h>
#include <ADC_util.h>

//*******************************
// RGB Strip Configuration
//*******************************
#define LED_PIN 17
#define LED_COUNT 24
#define SERIAL_TX_LED 0
#define SERIAL_RX_LED 1
#define LORA_TX_LED 2
#define LORA_RX_LED 3
#define BORDER_START_LED 4
#define BORDER_END_LED 23

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800); 
// RGB Strip Helped Functions
void clear_strip(uint32_t color);
void set_single_color(int pin, uint32_t color);
// RGB Strip Color Constants
const uint32_t serial_tx_color = strip.Color(255, 0, 0, 0);
const uint32_t serial_rx_color = strip.Color(255, 128, 0);
const uint32_t clear_color = strip.Color(0, 0, 0);
const uint32_t border_color = strip.Color(255, 0, 127);

//*******************************
// GPS Module Configuration
//*******************************
SFE_UBLOX_GNSS myGNSS;

boolean GNSS_enable = true;

long latitude_mdeg = 0;
long longitude_mdeg = 0;
int numSats = 0;

u_int32_t lastGPSTime = 0;

void publishPVTdata(UBX_NAV_PVT_data_t);

//*******************************
// Motor Drivers Configuration
//*******************************
int numMotors = 8;
int motorPWMPins[] = {23, 4, 6, 24, 25, 28, 29, 33};
int motorDIRPins[] = {22, 5, 7, 26, 27, 30, 31, 32};

int frleftMotorIdx = 0;
int frrightMotorIdx = 1;
int bkleftMotorIdx = 7;
int bkrightMotorIdx = 2;

boolean frleftMotorRev = true;
boolean frrightMotorRev = false;
boolean bkleftMotorRev = true;
boolean bkrightMotorRev = false;

int frleftMotorSpd = 0;
int frrightMotorSpd = 0;
int bkleftMotorSpd = 0;
int bkrightMotorSpd = 0;

int motorSpeed = 0;
int motorDir = 1;

//*******************************
// Battery Voltage Reader Config
//*******************************
#define BAT_SENS_PIN PIN_A2
ADC *adc = new ADC();              // adc object
float cells = 4;                   // Number of battery cells present
float minBatVoltage = 3.4 * cells; // Below this voltage motors should refuse to function
float maxBatVoltage = 4.2 * cells; // Battery voltage that should be considered fully charged

float batteryVoltage;
float batteryCharge;

u_int32_t lastBatteryTime = 0;
void publishBatteryData();

//*******************************
// Command Handling
//*******************************
void parseCommand(String command);
void setMotors(float magnitude, float direction);

// Sensor Reading Values
// Config Variables

void setup()
{
    // LED Ring
    strip.begin();
    strip.setBrightness(255);
    clear_strip(clear_color);
    
    // Serial Communications
    Serial.begin(115200);
    for (int i = 0; i < 3000 && !Serial; i++)
    {
        delay(10);
        //While waiting for the serial port to open, make a rainbow smiley face that changes color progressively faster and faster
        uint32_t rgbcolor = strip.gamma32(strip.ColorHSV((i*300*i/5000)%65536, 255, 255));
        strip.setPixelColor(0, rgbcolor);
        strip.setPixelColor(5, rgbcolor);
        for (int j = 5+6; j < LED_COUNT-5; j++)
        {
        strip.setPixelColor(j, rgbcolor);
        }
        strip.show();
    }
    
    // LoRA
    // I2C - For GPS Module
    Wire.begin();
    Wire.setClock(400000);

    // GPS Module
    if (!myGNSS.begin())
    {
        Serial.println("status:Ublox GPS not detected at default I2C address. Please check wiring.");
        GNSS_enable = false;
    } 
    else 
    {
        myGNSS.setI2COutput(COM_TYPE_UBX);                 //Set the I2C port to output UBX only (turn off NMEA noise)
        myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

        myGNSS.setNavigationFrequency(2); //Produce two solutions per second

        myGNSS.setAutoPVTcallback(&publishPVTdata);
    }

    // Prime Motors
    // Set motor pins to output
    for (int i = 0; i < numMotors; i++)
    {
        pinMode(motorPWMPins[i], OUTPUT);
        pinMode(motorDIRPins[i], OUTPUT);
    }  
    // Setup Battery Voltage Measurement
    // Set battery voltage sense pin to input
    pinMode(BAT_SENS_PIN, INPUT);

    // Configure ADC0
    adc->adc0->setAveraging(16);                                         // set number of averages
    adc->adc0->setResolution(16);                                        // set bits of resolution
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);          // change the sampling speed

    //Read battery voltage
    int value1 = adc->adc0->analogRead(BAT_SENS_PIN); // get raw ADC reading
    float batVoltage = value1 * 3.3 / (1.0 / (1.0 + 10.0)) / adc->adc0->getMaxValue(); // convert reading to voltage
    float batCharge = (batVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage); // convert voltage to naive charge percent (could be improved by factoring in discharge curves)

    clear_strip(clear_color);
}

void loop()
{
    //Read battery voltage and charge
    int value1 = adc->adc0->analogRead(BAT_SENS_PIN);
    batteryVoltage = value1 * 3.3 / (1.0 / (1.0 + 10.0)) / adc->adc0->getMaxValue();
    batteryCharge = (batteryVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage);

    // Process GPS
    if (millis() - lastGPSTime > 250 && GNSS_enable)
    {
        lastGPSTime = millis();
        myGNSS.checkUblox();     // Check for the arrival of new data and process it.
        myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
    }

    if (millis() - lastBatteryTime > 1000)
    {
        lastBatteryTime = millis();
        publishBatteryData();
    }

    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        parseCommand(command);
    }
    // Two Jobs
    //  - Relay Sensor Data
    //      - Read + Store Sensor Values
    //      - Package Sensor Values in Packet
    //      - Transmit Packet to Jetson and/or Radio
    //  - Read Commands
    //      - Unpack Commands
    //      - Execute Commands
    //      -
}

void parseCommand(String command)
{
    String exec = command.substring(0, command.indexOf(';'));
    if (exec.equals("set_motors"))
    {
        String mag = command.substring(exec.length()+1, command.indexOf(','));
        String dir = command.substring(command.indexOf(',')+1);

        Serial.println("response;" + exec + " " + mag + " " + dir);
    }
}

void clear_strip(uint32_t color)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void set_single_color(int pin, uint32_t color)
{
    strip.setPixelColor(pin, color);
    strip.show();
}

void publishPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{
    set_single_color(SERIAL_TX_LED, serial_tx_color);

    Serial.print("gps;");

    Serial.print("time=");
    uint8_t hms = ubxDataStruct.hour; // Print the hours
    Serial.print(hms);

    Serial.print(F(":"));
    hms = ubxDataStruct.min; // Print the minutes
    Serial.print(hms);

    Serial.print(F(":"));
    hms = ubxDataStruct.sec; // Print the seconds
    Serial.print(hms);

    Serial.print(F("."));
    unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds
    Serial.print(millisecs);

    Serial.print(",lat=");    
    long latitude = ubxDataStruct.lat; // Print the latitude
    Serial.print(latitude);

    Serial.print(",long=");
    long longitude = ubxDataStruct.lon; // Print the longitude
    Serial.print(longitude);

    long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level
    Serial.print(",alt=");
    Serial.print(altitude);

    long ground_speed = ubxDataStruct.gSpeed;
    Serial.print(",ground_speed=");
    Serial.print(ground_speed);

    long motion_heading = ubxDataStruct.headMot;
    Serial.print(",motion_heading=");
    Serial.print(motion_heading);

    long horizontal_accuracy = ubxDataStruct.hAcc;
    Serial.print(",horizontal_accuracy=");
    Serial.println(horizontal_accuracy);

    set_single_color(SERIAL_TX_LED, clear_color);
}

void publishBatteryData()
{
    set_single_color(SERIAL_TX_LED, serial_tx_color);

    Serial.print("battery;");

    Serial.print("v=");
    Serial.print(batteryVoltage);

    Serial.print(",c=");
    Serial.println(batteryCharge);

    set_single_color(SERIAL_TX_LED, clear_color);
}