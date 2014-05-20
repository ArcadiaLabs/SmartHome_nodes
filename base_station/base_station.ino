/*
- forwards rfm12b data to serial (source-payload-payload-payload-payload)
- forwards serial data to rfm12b (dest-payload-payload-payload-payload)
	- if dest = base, payloads are internal commands
                     
-----------------------------------------------------------------------------
 * Pin layout should be as follows:
 * Signal     Pin              Pin               Pin               Pin
 * ----------------------------------------------------------------------
 * SPI                                                             RFM12B
 * ----------------------------------------------------------------------
 * ?????      10                3                ??                1
 * SPI SCK    13                9                50                2
 * SPI MOSI   11                8                52                3
 *                                                                 
 * GND       GND              GND               GND                4
 * +3.3      VCC              VCC               VCC                5
 * ANT       ANT              ANT               ANT                6
 *                                                                 
 * SPI MISO   12                7                51                7
 * ?????       2                5                ??                8
*/

#include <JeeLib.h> // https://github.com/jcw/jeelib


// Fixed RF12 settings
#define MYNODE 30 //node ID of the receiver
#define freq RF12_433MHZ //frequency
#define group 210 //network group

//#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

MilliTimer sendTimer;
byte needToSend;
byte readyToSend;

 typedef struct {
          int rxnode
          int rx; 
          int rx1; 
          int rx2; 
          int rx3;
 } rxPayload;
 rxPayload rx;
 
  typedef struct {
          int txnode
          int tx; 
          int tx1; 
          int tx2; 
          int tx3; 
 } txPayload;
 txPayload tx;

 int nodeID; //node ID of tx, extracted from RF datapacket. Not transmitted as part of structure

byte byteRead;

unsigned long serialdata;
int inbyte;

int destnode;

int pinNumber;
int sensorVal;
int analogRate;
int digitalState;

void setup () {
  Serial.begin(9600);
//  Serial.println("RFM12 init");
  rf12_initialize(MYNODE, freq,group); // Initialise the RFM12B
  // Adjust low battery voltage to 2.2V UNTESTED!!!!!!!!!!!!!!!!!!!!!
  rf12_control(0xC040);
//  Serial.println("RFM12 init done");
  
}

void loop() {

 if (rf12_recvDone() && rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0) {
  nodeID = rf12_hdr & 0x1F; // get node ID
  rx = *(rxPayload*) rf12_data;
  int sourcenode = rx.destnode;
  int value = rx.rx;
  int value1 = rx.rx1;
  int value2 = rx.rx2;
  int value3 = rx.rx3;
  int rxsize = sizeof rx;
  
 if (RF12_WANTS_ACK) { // Send ACK if requested
   rf12_sendStart(RF12_ACK_REPLY, 0, 0);
 }
  blinkLED(rxLED, 1, 1);
  
  Serial.print(nodeID);Serial.print("/");
  Serial.print(value);Serial.print("/");
  Serial.print(value1);Serial.print("/");
  Serial.print(value2);Serial.print("/");
  Serial.print(value3);Serial.println("/"); 
  } 
   
  if (Serial.available()) {
    getSerial();
    destnode = serialdata;
    if (destnode == MYNODE) {
      internal_cmd();
    }
    else {
      destnode.tx = destnode;
      getSerial();
      tx.tx = serialdata;
      getSerial();
      tx.tx1 = serialdata;
      getSerial();
      tx.tx2 = serialdata;
      getSerial();
      tx.tx3 = serialdata;
      needToSend = 1;
    }
  }
  
  if (needToSend && rf12_canSend()) {
    needToSend = 0;
    readyToSend = 0;
    rf12_sendStart(destnode, &tx, sizeof tx);
 }
}

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//-------------------------------------------------------------------------------------------------
 static void rfwrite(){
  #ifdef USE_ACK
   for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
     rf12_sleep(-1);              // Wake up RF module
      while (!rf12_canSend())
      rf12_recvDone();
      rf12_sendStart(RF12_HDR_ACK, &tx, sizeof tx); 
      rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
      byte acked = waitForAck();  // Wait for ACK
      rf12_sleep(0);              // Put RF module to sleep
      if (acked) { return; }      // Return if ACK received
  
   Sleepy::loseSomeTime(RETRY_PERIOD * 1000);     // If no ack received wait and try again
   }
  #else
     rf12_sleep(-1);              // Wake up RF module
     while (!rf12_canSend())
     rf12_recvDone();
     rf12_sendStart(0, &tx, sizeof tx); 
     rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
     rf12_sleep(0);              // Put RF module to sleep
     return;
  #endif
 }

void blinkLED(byte targetPin, int numBlinks, int blinkRate) {
  for (int i=0; i < numBlinks; i++) {
    digitalWrite(targetPin, HIGH);   // sets the LED on
    delay(blinkRate);                     // waits for blinkRate milliseconds
    digitalWrite(targetPin, LOW);    // sets the LED off
    delay(blinkRate);
  }
}

void internal_cmd() {
  getSerial();
  int cmd = serialdata;
  switch(cmd) {
    
    // PIN WRITE
    case 1: {
      getSerial();
      int data1 = serialdata;
      switch (data1) {
        
        // DIGITAL WRITE
        case 1: {
          getSerial();
          pinNumber = serialdata;
          getSerial();
          digitalState = serialdata;
          pinMode(pinNumber, OUTPUT);
          if (digitalState == 0) {
            digitalWrite(pinNumber, LOW);
          }
          if (digitalState == 1) {
            digitalWrite(pinNumber, HIGH);
          }

          Serial.print(destnode);Serial.print("/");
          Serial.print(cmd);Serial.print("/");
          Serial.print(data1);Serial.print("/");
          Serial.print(pinNumber);Serial.print("/");
          Serial.print(digitalState);Serial.println("/"); 
          pinNumber = 0;
          break;
        }
        
        // ANALOG WRITE
        case 2: {
          getSerial();
          pinNumber = serialdata;
          getSerial();
          analogRate = serialdata;
          pinMode(pinNumber, OUTPUT);
          analogWrite(pinNumber, analogRate); 
          
          Serial.print(destnode);Serial.print("/");
          Serial.print(cmd);Serial.print("/");
          Serial.print(data1);Serial.print("/");
          Serial.print(pinNumber);Serial.print("/");
          Serial.print(analogRate);Serial.println("/");
          pinNumber = 0;
          break;
        }               
      }
      break;
    }
    
    // PIN READ
    case 2:{
      getSerial();
      int data1 = serialdata;
      switch (data1) {
        
        // DIGITAL READ
        case 1: {
          getSerial();
          pinNumber = serialdata;
          getSerial();
          int nulldata = serialdata;
          pinMode(pinNumber, INPUT);
          sensorVal = digitalRead(pinNumber);
          Serial.println(sensorVal);
          sensorVal = 0;
          pinNumber = 0;
          break;
        }
        
        // ANALOG READ
        case 2: {         
          getSerial();
          pinNumber = serialdata;
          getSerial();
          int nulldata = serialdata;
          pinMode(pinNumber, INPUT);
          sensorVal = analogRead(pinNumber);
          Serial.println(sensorVal);
          sensorVal = 0;
          pinNumber = 0;
          break;
        }
      }
    }
    break;
  }
}

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    // inbyte = Serial.read()-'0';
    if (inbyte > 0 && inbyte != '/')
    {
     
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}
