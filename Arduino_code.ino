#include <Arduino.h>
#include "wiring_private.h"
#include <Adafruit_GPS.h>
#include <genieArduino.h>

Uart dispSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Adafruit_GPS GPS(&Serial1);
#define GPSECHO  false

Genie genie;

bool BST = false; 
bool setupT = true;
int currentSpeed, prevSpeed = 0;
String currentTime = "00:00";
//String currentHour, currentMin = "00";
int currentHour, currentMin, lastMin;
int distance = 0;

uint32_t timer;

int ack=0;
//Attach the interupt handler to the SERCOM
void SERCOM0_Handler()
{
  dispSerial.IrqHandler();
}

int BSTpin = 2;
int RSTpin = 3;

void setup() {
  // Reassign pins 5 and 6 to SERCOM alt for screen serial
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);

  //start serial
  dispSerial.begin(9600);
  genie.Begin(dispSerial);

  Serial.begin(115200);
  GPS.begin(9600);
  delay(5000);
  Serial.println("Starting");


  timer = millis();


  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  pinMode(BSTpin, INPUT_PULLUP);
  pinMode(RSTpin, INPUT_PULLUP);

  //check is BST is required
  if (digitalRead(BSTpin) == LOW){
    BST = true;
    Serial.println("BST active");
  }

}


void loop() { 
char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  //check is Reset of odometer is required
  if (digitalRead(RSTpin) == LOW){
    distance = 0;
    Serial.println("Reset distance");
  }
  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    if (setupT != true){
      lastMin=currentMin;
    }
    currentHour = GPS.hour;
    currentMin = GPS.minute;
    if (BST == true){
      currentHour = (currentHour +1)%24;
    }
    Serial.print(currentHour);
    Serial.print(":");
    Serial.println(currentMin);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    
    if (GPS.fix) {
      genie.WriteObject(GENIE_OBJ_LED,0, 0);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      currentSpeed = round(GPS.speed*1.151);
      distance = distance+((currentSpeed*10000)/3600);
      Serial.print("Distance: ");
      Serial.println(distance);

    }else{
      genie.WriteObject(GENIE_OBJ_LED,0, 1);
      //genie.WriteObject(GENIE_OBJ_LED,0, 0);
      Serial.println("No fix");
      currentSpeed = 0;
      genie.WriteObject(GENIE_OBJ_IANGULAR_METER,0, currentSpeed);
    }
  //display time
  if ((currentMin != lastMin) || (setupT == true)){
    char timebuff[6];
    snprintf(timebuff,6,"%02d:%02d",currentHour, currentMin);
    //currentTime = String(currentHour + ":" + currentMin);
    genie.WriteStr(0,timebuff);
    setupT=false;
  }
  //display speed
  if (currentSpeed <0){
    currentSpeed =0;
  }else if (currentSpeed >30){
    currentSpeed = 30;
  }
  genie.WriteObject(GENIE_OBJ_IANGULAR_METER,0, currentSpeed);

  genie.WriteObject(GENIE_OBJ_LED_DIGITS,0, (distance/100));
  }
}
