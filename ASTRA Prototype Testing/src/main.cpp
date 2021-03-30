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
#include <communication_standard.h>

// Declare Global Variables
#define LED_PIN 17
#define LED_COUNT 24

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800); 

// Sensor Reading Values
// Config Variables

void setup()
{
    // LED Ring
    strip.begin();
    strip.setBrightness(255);
    for (int i = 0; i < LED_COUNT; i++)
    {
        strip.setPixelColor(i, strip.ColorHSV(
            (int)(i * (65536/LED_COUNT)),
            255,
            255
            ));
    }
    strip.show();
    
    // Serial Communications
    Serial.begin(115200);
    
    
    // LoRA
    // I2C
    // GPS Module
    // Prime Motors
    // Setup Battery Voltage Measurement
}

void loop()
{
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