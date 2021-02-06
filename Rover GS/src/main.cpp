#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915E6

String outgoing;

byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xFF; // address of this device
byte destination = 0xBB;  // destination to send to
long lastSendTime = 0;    // last send time
int interval = 2000;      // interval between sends

void sendMessage(String);
boolean onReceive(int);

void setup()
{
  Serial.begin(115200); // initialize serial
  while (!Serial)
    ;

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT); // set CS, reset, IRQ pin

  if (!LoRa.begin(RF95_FREQ))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ; // if failed, do nothing
  }
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();

  Serial.println("LoRa init succeeded.");
  Serial.setTimeout(25);
}

void loop()
{
  if (Serial.available() > 1)
  {
    // read the incoming byte:
    
    String message = Serial.readString(); // send a message
    sendMessage(message);
    Serial.println("rcv");
  }

  //onReceive(LoRa.parsePacket());
  delay(15);
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

boolean onReceive(int packetSize)
{
  if (packetSize == 0)
    return false; // if there's no packet, return

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
    return false; // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF)
  {
    Serial.println("This message is not for me.");
    return false; // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  return true;
}