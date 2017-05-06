// Code for controlling thrust vanes on MK2 Inconel engine

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <avr/pgmspace.h>
#include <Telemetry.h>
#include <HX711.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

#define LED           10        // LED signals errors
#define servo1Pin     A0        // pins for controlling servos
#define servo2Pin     A1
#define servo3Pin     A3
#define servo4Pin     A2

#define forcePin1           13        // pins reading load cell
#define forcePin2           12        // load cell 2 has issues
#define forcePin3           11
#define forcePin4           10
#define CLK                 9
#define force1Offset        122800
#define force2Offset        8388607   // will set not working output to 0
#define force3Offset        507600
#define force4Offset        428700
#define force1Scale         5/106000
#define force2Scale         1//5/400000
#define force3Scale         5/430000
#define force4Scale         5/440000
/*HX711 scale1(forcePin1, CLK);         // for load cells, hangs without load cell connected
HX711 scale2(forcePin2, CLK);
HX711 scale3(forcePin3, CLK);
HX711 scale4(forcePin4, CLK);*/
float force1, force2, force3, force4;

// Offsets to make servos align vertically at v=90
#define servo1Offset  90        // for MG995 #1
#define servo2Offset  84        // for MG995 #2
#define servo3Offset  83
#define servo4Offset  83
#define vMax          20        // max angular deflection (avoids stall)
int v1 = 0, v2 = 0, v3 = 0, v4 = 0;
//int v1 = servo1Offset, v2 = servo2Offset, v3 = servo3Offset, v4 = servo4Offset;

int v1inc = 10;
int v2inc = v1inc;

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

long time1, time2;
long testLength = 1000;


void setup() {
  Serial.begin(38400, SERIAL_8N2);    Serial.println("Starting...");
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  servo1.write(servo1Offset);
  servo2.write(servo2Offset);
  servo3.write(servo3Offset);
  servo4.write(servo4Offset);
  
  //scale.set_scale(20400);                           // Value obtained using SparkFun_HX711_Calibration sketch
  //scale.tare();                                     // Resets the scale to 0, assumes no weight at start up
  
  if (!RTC.isrunning()) { RTC.adjust(DateTime(__DATE__, __TIME__)); }
}


void loop() {
  long time0 = millis();// - testRestart;
  
  // TESTS TO PERFORM ////////////////////
  // Tandem All
  if      (time0 > testLength*60 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*59 ) {
        v1 = -vMax;       v2 = -vMax;
        v3 = -vMax;       v4 = -vMax;}
  else if (time0 > testLength*58 ) {
        v1 = vMax;        v2 = vMax;
        v3 = vMax;        v4 = vMax; }
  else if (time0 > testLength*57 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  
  // Tandem 2,4
  else if (time0 > testLength*56 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*55 ) {
        v1 = 0;           v2 = vMax;
        v3 = 0;           v4 = -vMax; }
  else if (time0 > testLength*54 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*53 ) {
        v1 = 0;           v2 = -vMax;
        v3 = 0;           v4 = vMax; }
  else if (time0 > testLength*52 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*51 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*50 ) {
        v1 = 0;           v2 = -vMax;
        v3 = 0;           v4 = -vMax; }
  else if (time0 > testLength*49 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*48 ) {
        v1 = 0;           v2 = vMax;
        v3 = 0;           v4 = vMax; }
  else if (time0 > testLength*47 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }

  // Tandem 1,3
  else if (time0 > testLength*46 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*45 ) {
        v1 = vMax;        v2 = 0;
        v3 = -vMax;       v4 = 0; }
  else if (time0 > testLength*44 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*43 ) {
        v1 = -vMax;       v2 = 0;
        v3 = vMax;        v4 = 0; }
  else if (time0 > testLength*42 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*41 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*40 ) {
        v1 = -vMax;       v2 = 0;
        v3 = -vMax;       v4 = 0; }
  else if (time0 > testLength*39 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*38 ) {
        v1 = vMax;        v2 = 0;
        v3 = vMax;        v4 = 0; }
  else if (time0 > testLength*37 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
        
  // Single 4
  else if (time0 > testLength*36 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*35 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = -vMax; }
  else if (time0 > testLength*34 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = vMax; }
  else if (time0 > testLength*33 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*32 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*31 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = -vMax; }
  else if (time0 > testLength*30 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*29 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = vMax; }
  else if (time0 > testLength*28 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
        
  // Single 3
  else if (time0 > testLength*27 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*26 ) {
        v1 = 0;           v2 = 0;
        v3 = -vMax;       v4 = 0; }
  else if (time0 > testLength*25 ) {
        v1 = 0;           v2 = 0;
        v3 = vMax;        v4 = 0; }
  else if (time0 > testLength*24 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*23 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*22 ) {
        v1 = 0;           v2 = 0;
        v3 = -vMax;       v4 = 0; }
  else if (time0 > testLength*21 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*20 ) {
        v1 = 0;           v2 = 0;
        v3 = vMax;        v4 = 0; }
  else if (time0 > testLength*19 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
        
  // Single 2
  else if (time0 > testLength*18 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*17 ) {
        v1 = 0;           v2 = -vMax;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*16 ) {
        v1 = 0;           v2 = vMax;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*15 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*14 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*13 ) {
        v1 = 0;           v2 = -vMax;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*12 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*11 ) {
        v1 = 0;           v2 = vMax;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*10 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }

  // Single 1
  else if (time0 > testLength*9 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*8 ) {
        v1 = -vMax;       v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*7 ) {
        v1 = vMax;        v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*6 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*5 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*4 ) {
        v1 = -vMax;       v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*3 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*2 ) {
        v1 = vMax;        v2 = 0;
        v3 = 0;           v4 = 0; }
  else if (time0 > testLength*1 ) {
        v1 = 0;           v2 = 0;
        v3 = 0;           v4 = 0; }
  
  Serial.print(v1); Serial.print("  "); Serial.print(v2); Serial.print("  "); Serial.print(v3); Serial.print("  "); Serial.println(v4);
  ///////////////////////////////////////////////////
  
  /*// Simulates changing servo angle setting
  if      (v1 >= servo1Offset+vMax) { v1inc = -v1inc; }   
  else if (v1 <= servo1Offset-vMax) { v1inc = -v1inc; }
  if      (v2 >= servo2Offset+vMax) { v2inc = -v2inc; }
  else if (v2 <= servo2Offset-vMax) { v2inc = -v2inc; }
  v1 = v1 + v1inc;
  v2 = v2 + v2inc;
  v3 = v3 + v1inc;
  v4 = v4 + v2inc;
  servo1.write(v1);
  servo2.write(v2);
  servo3.write(v3);
  servo4.write(v4);*/
  
  servo1.write(v1 + servo1Offset);
  servo2.write(v2 + servo2Offset);
  servo3.write(v3 + servo3Offset);
  servo4.write(v4 + servo4Offset);
  
  /*force1= scale1.get_units();              // Forces (converted to pounds by force1Scale
  force2= scale2.get_units();
  force3= scale3.get_units();
  force4= scale4.get_units();
  Serial.print("force1:"); Serial.print((force1-force1Offset)*force1Scale);
  Serial.print("\t"); Serial.print("force2:"); Serial.print((force2-force2Offset)*force2Scale);
  Serial.print("\t"); Serial.print("force3:"); Serial.print((force3-force3Offset)*force3Scale);
  Serial.print("\t"); Serial.print("force4:"); Serial.println((force4-force4Offset)*force4Scale);
  */
  
  /*// Downlink
  BEGIN_SEND
  SEND_ITEM(servo1_set, v1);                // 4 ms
  SEND_ITEM(servo2_set, v2);                // 4 ms
  SEND_ITEM(servo3_set, v3);
  SEND_ITEM(servo4_set, v4);
  SEND_ITEM(servo1_fb, fb1);
  SEND_ITEM(servo2_fb, fb2);
  SEND_ITEM(servo3_fb, fb3);
  SEND_ITEM(servo4_fb, fb4);
  //SEND_ITEM(force1, force1);
  //SEND_ITEM(force2, force2);
  //SEND_ITEM(force3, force3);
  //SEND_ITEM(force4, force4);
  END_SEND*/

}
