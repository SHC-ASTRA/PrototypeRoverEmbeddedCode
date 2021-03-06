#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <ADC.h>
#include <ADC_util.h>

#define LED_PIN 17
#define LED_COUNT 24

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800);

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 14

#define RF95_FREQ 915E6

#define BAT_SENS_PIN PIN_A2

#define SerialJ Serial1

#define NX_SLEEP_PIN 3
#define NX_RESET_PIN 2

ADC *adc = new ADC();
;                                  // adc object
float cells = 4;                   // Number of battery cells present
float minBatVoltage = 3.4 * cells; // Below this voltage motors should refuse to function
float maxBatVoltage = 4.2 * cells; // Battery voltage that should be considered fully charged

String outgoing;

byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xBB; // address of this device
byte destination = 0xFF;  // destination to send to

void sendMessage(String);
String onReceive(int);

SFE_UBLOX_GNSS myGNSS;

boolean GNSS_enable = true;

long latitude_mdeg = 0;
long longitude_mdeg = 0;
int numSats = 0;

u_int32_t lastGPSTime = 0;

void printPVTdata(UBX_NAV_PVT_data_t);

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

uint32_t lastPacketTime = 0;

void setup()
{
  //Start by initializing RGB LED Ring
  strip.begin();
  strip.setBrightness(32);

  //Draw a red smiley face
  uint32_t red = strip.gamma32(strip.ColorHSV(0, 255, 255));
  strip.setPixelColor(0, red);
  strip.setPixelColor(5, red);
  for (int i = 5+6; i < LED_COUNT-5; i++)
  {
    strip.setPixelColor(i, red);
  }
  strip.show();

  //Initialize the serial ports
  Serial.begin(115200);
  SerialJ.begin(115200,SERIAL_8N1);

  //Wait 30 seconds for the USB serial port to connect to a computer
  //If 30 seconds passes with no connection, proceed anyways
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

  // Initialize LoRa radio
  Serial.println("Initializing LoRa");
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(RF95_FREQ))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ; // if failed, do nothing
        // TODO: change this to more gracefully deal with failure
  }
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("LoRa init succeeded.");

  // Initialize I2C for the GPS module
  Wire.begin();
  Wire.setClock(400000);

  // Initialize the GPS module
  if (!myGNSS.begin())
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
    GNSS_enable = false;  // Set a flag to false if the GPS is not functional
  }
  else
  {
    myGNSS.setI2COutput(COM_TYPE_UBX);                 //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    myGNSS.setNavigationFrequency(2); //Produce two solutions per second

    //myGNSS.setAutoPVTcallback(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
  }

  // Set motor pins to output
  for (int i = 0; i < numMotors; i++)
  {
    pinMode(motorPWMPins[i], OUTPUT);
    pinMode(motorDIRPins[i], OUTPUT);
  }

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

  Serial.printf("Bat Charge: %.2f   Bat Voltage: %.2f\n\r",batCharge,batVoltage);

  // pinMode(NX_SLEEP_PIN, OUTPUT);

  // digitalWrite(NX_SLEEP_PIN, HIGH);
  // delay(500);
  // digitalWrite(NX_SLEEP_PIN, LOW);

}

void loop()
{
  //Read battery voltage
  int value1 = adc->adc0->analogRead(BAT_SENS_PIN);
  float batVoltage = value1 * 3.3 / (1.0 / (1.0 + 10.0)) / adc->adc0->getMaxValue();
  float batCharge = (batVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage);

  //Light up LED ring to correspond to battery charge level
  for (int i = 0; i < LED_COUNT; i++)
  {
    // uint32_t rgbcolor = strip.gamma32(strip.ColorHSV((i * 65536) / (LED_COUNT + 1) + millis() * 100, 255, 255));

    int valueVar = (int)constrain(((batCharge*LED_COUNT)-i)*255,0,255);

    uint32_t rgbcolor = strip.gamma32(strip.ColorHSV((int)map(batCharge,1.0,0.0,1,65536/3), 255, valueVar));
    strip.setPixelColor(i, rgbcolor);
  }
  strip.show();

  delay(1);

  // Check for LoRa messages
  String message = "";
  if (!((message = onReceive(LoRa.parsePacket())).equals("")))
  {

    // Parse message to control commands
    message.trim();

    int horz = message.substring(1, message.indexOf(',')).toInt();
    int vert = message.substring(message.indexOf('V') + 1, message.indexOf(',', message.indexOf('V'))).toInt();

    Serial.print(horz);
    Serial.print(", ");
    Serial.println(vert);

    // Generate motor speeds from control commands
    frleftMotorSpd = constrain(-vert - horz, -255, 255);
    frrightMotorSpd = constrain(-vert + horz, -255, 255);
    bkleftMotorSpd = constrain(-vert - horz, -255, 255);
    bkrightMotorSpd = constrain(-vert + horz, -255, 255);

    Serial.print(frleftMotorSpd);
    Serial.print(", ");
    Serial.print(bkleftMotorSpd);
    Serial.print(", ");
    Serial.print(frrightMotorSpd);
    Serial.print(", ");
    Serial.println(bkrightMotorSpd);

    lastPacketTime = millis();

    //sendMessage(buffer);
    //Serial.print("Sending ");
    //Serial.print(buffer);
  }


  // If no LoRa message has been recieved for 0.25 seconds or battery voltage is too low,
  // set motor speeds to zero
  if (millis() - lastPacketTime > 250 || batVoltage < minBatVoltage)
  {
    frleftMotorSpd = 0;
    frrightMotorSpd = 0;
    bkleftMotorSpd = 0;
    bkrightMotorSpd = 0;
  }

  // Set motor speeds
  analogWrite(motorPWMPins[frleftMotorIdx], abs(frleftMotorSpd));
  digitalWrite(motorDIRPins[frleftMotorIdx], frleftMotorRev ? frleftMotorSpd > 0 : frleftMotorSpd < 0);

  analogWrite(motorPWMPins[frrightMotorIdx], abs(frrightMotorSpd));
  digitalWrite(motorDIRPins[frrightMotorIdx], frrightMotorRev ? frrightMotorSpd > 0 : frrightMotorSpd < 0);

  analogWrite(motorPWMPins[bkleftMotorIdx], abs(bkleftMotorSpd));
  digitalWrite(motorDIRPins[bkleftMotorIdx], bkleftMotorRev ? bkleftMotorSpd > 0 : bkleftMotorSpd < 0);

  analogWrite(motorPWMPins[bkrightMotorIdx], abs(bkrightMotorSpd));
  digitalWrite(motorDIRPins[bkrightMotorIdx], bkrightMotorRev ? bkrightMotorSpd > 0 : bkrightMotorSpd < 0);

  // Process GPS
  if (millis() - 250 > lastGPSTime && GNSS_enable)
  {
    lastGPSTime = millis();
    myGNSS.checkUblox();     // Check for the arrival of new data and process it.
    myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
  }
}

void printPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{
  Serial.println();

  Serial.print(F("Time: "));        // Print the time
  uint8_t hms = ubxDataStruct.hour; // Print the hours
  if (hms < 10)
    Serial.print(F("0")); // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F(":"));
  hms = ubxDataStruct.min; // Print the minutes
  if (hms < 10)
    Serial.print(F("0")); // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F(":"));
  hms = ubxDataStruct.sec; // Print the seconds
  if (hms < 10)
    Serial.print(F("0")); // Print a leading zero if required
  Serial.print(hms);
  Serial.print(F("."));
  unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds
  if (millisecs < 100)
    Serial.print(F("0")); // Print the trailing zeros correctly
  if (millisecs < 10)
    Serial.print(F("0"));
  Serial.print(millisecs);

  long latitude = ubxDataStruct.lat; // Print the latitude
  Serial.print(F(" Lat: "));
  Serial.print(latitude);

  long longitude = ubxDataStruct.lon; // Print the longitude
  Serial.print(F(" Long: "));
  Serial.print(longitude);
  Serial.print(F(" (degrees * 10^-7)"));

  long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level
  Serial.print(F(" Height above MSL: "));
  Serial.print(altitude);
  Serial.println(F(" (mm)"));
}

void sendMessage(String outgoing)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(destination);       // add destination address
  LoRa.write(localAddress);      // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}

String onReceive(int packetSize)
{
  if (packetSize == 0)
    return ""; // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();       // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return ""; // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF)
  {
    Serial.println("This message is not for me.");
    return ""; // skip rest of function
  }

  // // if message is for this device, or broadcast, print details:
  // Serial.println("Received from: 0x" + String(sender, HEX));
  // Serial.println("Sent to: 0x" + String(recipient, HEX));
  // Serial.println("Message ID: " + String(incomingMsgId));
  // Serial.println("Message length: " + String(incomingLength));
  // Serial.println("Message: " + incoming);
  // Serial.println("RSSI: " + String(LoRa.packetRssi()));
  // Serial.println("Snr: " + String(LoRa.packetSnr()));
  // Serial.println();
  return incoming;
}