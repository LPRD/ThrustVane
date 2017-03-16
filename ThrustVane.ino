// Code for controlling thrust vanes on MK2 Inconel engine

#define testing 0               // Change to 1 before testing / flight    !!!!!!!!!!!!!!
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/pgmspace.h>
#include <Telemetry.h>
#include <HX711.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

#define LED           10        // LED signals errors
#define servo1Pin     2         // pins for controlling servos
#define servo2Pin     3
#define servo3Pin     0
#define servo4Pin     1
#define feedback1     4         // pins reading servo feedback
#define feedback2     5
#define feedback3     6
#define feedback4     7

#define DOUT          8         // pins reading load cell
#define CLK           9
HX711 scale(DOUT, CLK);
float force1, force2, force3, force4;

#define loopDelay     200                     // min 200 with SD enabled
#define dataTime ((float)loopDelay)/1000      // time between data
#define SDdelay       20
#define flagIncrement 10
#define sdErrorLimit  2         // # of times SD card access can fail before it is no longer attempted
int flag = 0;       long checkSD;

// Offsets to make servos align vertically at v=90
#define servo1Offset  90        // for MG995 #1
#define servo2Offset  90        // for MG995 #2
#define servo3Offset  90
#define servo4Offset  90
#define vMax          20        // max angular deflection (avoids stall)
int v1 = servo1Offset;
int v2 = servo2Offset;
int v3 = servo3Offset;
int v4 = servo4Offset;
int fb1, fb2, fb3, fb4;

int v1inc = 2;
int v2inc = 2;

#define SEND_VECTOR_ITEM(field, value)\
  SEND_ITEM(field, value.x())         \
  SEND_GROUP_ITEM(value.y())          \
  SEND_GROUP_ITEM(value.z())

#define WRITE_CSV_ITEM(value)         \
  dataFile.print(F(", ")); dataFile.print(value);

#define WRITE_CSV_VECTOR_ITEM(value)  \
  WRITE_CSV_ITEM(value.x())           \
  WRITE_CSV_ITEM(value.y())           \
  WRITE_CSV_ITEM(value.z())

unsigned int missed_deadlines = 0;
long time1, time2;


void setup() {
  if (testing) { pinMode(LED,OUTPUT); }             // makes LED flash brightly
  pinMode(feedback1,INPUT);
  pinMode(feedback2,INPUT);
  pinMode(feedback3,INPUT);
  pinMode(feedback4,INPUT);
  
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  
  servo1.write(v1+servo1Offset);
  servo2.write(v2+servo2Offset);
  servo3.write(v3+servo3Offset);
  servo4.write(v4+servo4Offset);
  
  Serial.begin(38400, SERIAL_8N2);    Serial.println();
  scale.set_scale(20400);                           // Value obtained using SparkFun_HX711_Calibration sketch
  scale.tare();                                     // Resets the scale to 0, assumes no weight at start up
  
  while (!bno.begin()) {                            // flashes to signal error
    Serial.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  }
  if (!RTC.isrunning()) { RTC.adjust(DateTime(__DATE__, __TIME__)); }
  if (!SD.begin(10)) { Serial.println(F("SD err")); }
  else {                                            // generates file name
    for (uint16_t nameCount = 0; nameCount < 1000; nameCount++) {
      filename[4] = nameCount/100 + '0';
      filename[5] = (nameCount%100)/10 + '0';
      filename[6] = nameCount%10 + '0';
      if (!SD.exists(filename)) {                   // opens if file doesn't exist
        dataFile = SD.open(filename, FILE_WRITE);
        Serial.print(F("\twriting "));
        Serial.println(filename);
        dataFile.println(F("abs time,sys date,sys time,servo1_set,servo2_set,servo3_set,servo4_set,servo1_fb,servo2_fb,servo3_fb,servo4_fb,force1,force2,force3,force4"));
        //dataFile.println(F("abs time,sys date,sys time,servo1_set,servo2_set,servo1_fb,servo2_fb,force1"));
        break;
      }
    }
  }
}


void loop() {
  long time0 = millis();
  
  // Simulates changing servo angle setting
  if      (v1 >= servo1Offset+vMax) { v1inc = -v1inc; }   
  else if (v1 <= servo1Offset-vMax) { v1inc = -v1inc; }
  if      (v2 >= servo2Offset+vMax) { v2inc = -v2inc; }
  else if (v2 <= servo2Offset-vMax) { v2inc = -v2inc; }
  v1 = v1 + v1inc;
  v2 = v2 + v2inc;
  
  servo1.write(v1);
  servo2.write(v2);
  servo3.write(v3);
  servo4.write(v4);
  fb1 = analogRead(feedback1)-servo1Offset;
  fb2 = analogRead(feedback2)-servo2Offset;
  fb3 = analogRead(feedback3)-servo3Offset;
  fb4 = analogRead(feedback4)-servo4Offset;
  
  //force1= scale.get_units();              // Forces in lb
  //force2= scale.get_units();
  //force3= scale.get_units();
  //force4= scale.get_units();
  
  // Downlink
  BEGIN_SEND
  SEND_ITEM(servo1_set, v1);                // 4 ms
  SEND_ITEM(servo2_set, v2);                // 4 ms
  SEND_ITEM(servo3_set, v3);
  SEND_ITEM(servo4_set, v4);
  
  SEND_ITEM(servo1_fb, fb1);
  SEND_ITEM(servo2_fb, fb2);
  SEND_ITEM(servo3_fb, fb3);
  SEND_ITEM(servo4_fb, fb4);
  SEND_ITEM(force1, force1);
  //SEND_ITEM(force2, force2);
  //SEND_ITEM(force3, force3);
  //SEND_ITEM(force4, force4);
  END_SEND
  
// Writing to SD Card
  if ((flag<flagIncrement*sdErrorLimit-sdErrorLimit)&&(flag>0))   { flag--; }
  if ((flag<flagIncrement*sdErrorLimit-sdErrorLimit)&&(dataFile)) {
    DateTime now = RTC.now();           checkSD = millis();     // checks for SD removal
    dataFile.print(millis());           dataFile.print(',');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.year()  ,DEC);   dataFile.print('/');
    dataFile.print(now.month() ,DEC);   dataFile.print('/');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.day()   ,DEC);   dataFile.print(',');
    dataFile.print(now.hour()  ,DEC);   dataFile.print(':');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.minute(),DEC);   dataFile.print(':');
    dataFile.print(now.second(),DEC);
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_ITEM(v1)
    WRITE_CSV_ITEM(v2)
    WRITE_CSV_ITEM(v3)
    WRITE_CSV_ITEM(v4)
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_ITEM(fb1)
    WRITE_CSV_ITEM(fb2)
    WRITE_CSV_ITEM(fb3)
    WRITE_CSV_ITEM(fb4)
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_ITEM(force1)
    WRITE_CSV_ITEM(force2)
    WRITE_CSV_ITEM(force3)
    WRITE_CSV_ITEM(force4)
    timedout:
    dataFile.println();     dataFile.flush();
  }
  
  if (time0 + loopDelay < millis()) {
    Serial.print(F("Schedule err: "));
    Serial.println(time0 + loopDelay - (signed long)millis());
    missed_deadlines++;
    SEND(missed_deadlines, missed_deadlines);
  }
  else {
    if(flag>=flagIncrement*sdErrorLimit-sdErrorLimit) { digitalWrite(LED,LOW); }
    delay(time0 + loopDelay - millis());    // continuously adjusted for desired dataTime
    digitalWrite(LED,HIGH);
  }
  
}
