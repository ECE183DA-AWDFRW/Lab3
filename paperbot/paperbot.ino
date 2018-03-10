/*
  Wireless Servo Control, with ESP as Access Point
  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.
  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP
  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon
    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system
  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis
*/
#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"

#include <Wire.h>

#include <VL53L0X.h>

// Magnetometer stuff
#define   MPU9250_ADDRESS           0x68
#define   MAG_ADDRESS               0x0C

#define   GYRO_FULL_SCALE_250_DPS   0x00  
#define   GYRO_FULL_SCALE_500_DPS   0x08
#define   GYRO_FULL_SCALE_1000_DPS  0x10
#define   GYRO_FULL_SCALE_2000_DPS  0x18

#define   ACC_FULL_SCALE_2_G        0x00  
#define   ACC_FULL_SCALE_4_G        0x08
#define   ACC_FULL_SCALE_8_G        0x10
#define   ACC_FULL_SCALE_16_G       0x18

int box_a = 0; //angle/heading
int box_x = 3000; //box width
int box_y = 2000; //box height

// Callibration values. These values experimentally found for my room. 
int16_t mag_bias_x = 31;
int16_t mag_bias_y = 26;
int16_t mag_bias_z = 51;
float   mag_SI_x = 1.16;
float   mag_SI_y = 1.38;
float   mag_SI_z = 0.71;

// Range Sensor Things
VL53L0X sensor_y;  // y-dir
VL53L0X sensor_x; // x-dir

// Common to all
#define   SDA_PORT 14
#define   SCL_PORT 12

// Servo things
const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

// WiFi AP parameters
char* ap_ssid = "ARDLab3_AP";
char* ap_password = "Hello";

// WiFi STA parameters. Not Used.
char* sta_ssid = 
  "ARDLab3_STA";
char* sta_password = 
  "Hello";

char* mDNS_name = "paperbot";

String html;
String css;

//
// I2C Helper Functions //
//

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    
    // Read Nbytes
    Wire.requestFrom(Address, Nbytes); 
    uint8_t index=0;
    while (Wire.available())
      Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

//
// Setup and Loop //
//
void setup() {
    Serial.begin(115200);
    DEBUG("Started serial.");
    Wire.begin(SDA_PORT,SCL_PORT);
    
    setupServos();
    setupMagnetometer();
    setupRange();  

    // Setup Wifi
    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop_s();
    LED_OFF;
}

void loop() {
    wsLoop();
    httpLoop();
}


//
// Movement Functions //
//

void drive(int left, int right) {
    servo_left.write(left);
    servo_right.write(right);
}

void stop_s() {
    DEBUG("stop");
    drive(servo_left_ctr, servo_right_ctr);
    LED_OFF;
}

void forward() {
    DEBUG("forward");
    drive(180, 0);
}

void backward() {
    DEBUG("backward");
    drive(0, 180);
}

void right() {
    DEBUG("left");
    drive(180, 180);
}

void left() {
    DEBUG("right");
    drive(0, 0);
}

//
// Setup //
//
void setupServos() {

    // setup LEDs and Motors
    pinMode(LED_PIN, OUTPUT);  
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

void setupMagnetometer(){
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
}

void setupRange(){
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    digitalWrite(D7, LOW);
    digitalWrite(D8, LOW);
    delay(100);
    
    digitalWrite(D3, HIGH);
    delay(150);
    Serial.println("00");
    sensor_y.init(true);
    Serial.println("01");
    delay(100);
    sensor_y.setAddress((uint8_t)22);

    digitalWrite(D4, HIGH);
    delay(150);
    sensor_x.init(true);
    Serial.println("03");
    delay(100);
    sensor_x.setAddress((uint8_t)25);
    Serial.println("04");
    
    Serial.println("addresses set");

    Serial.println ("I2C scanner. Scanning ...");
    byte count = 0;
  
    for (byte i = 1; i < 120; i++)
    {
  
      Wire.beginTransmission (i);
      if (Wire.endTransmission () == 0)
      {
        Serial.print ("Found address: ");
        Serial.print (i, DEC);
        Serial.print (" (0x");
        Serial.print (i, HEX);
        Serial.println (")");
        count++;
        delay (1);  // maybe unneeded?
      } // end of good response
    } // end of for loop
    Serial.println ("Done.");
    Serial.print ("Found ");
    Serial.print (count, DEC);
    Serial.println (" device(s).");
    delay(3000);
}

void readRange(int& range_x, int& range_y){
    range_x = sensor_x.readRangeSingleMillimeters() + (uint8_t)29;
    if(sensor_x.timeoutOccurred()){
      range_x = -1;
    }
    range_y = sensor_y.readRangeSingleMillimeters() + (uint8_t)13;
    if(sensor_y.timeoutOccurred()){
      range_y = -1;
    }    
}

void readMag(int16_t &mx, int16_t &my, int16_t &mz){
    // _____________________
    // :::  Magnetometer ::: 
  
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
    
    // Read register Status 1 and wait for the DRDY: Data Ready
    uint8_t ST1;
    do
    {
      I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));
    // Read magnetometer data  
    uint8_t Mag[7];  
    I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
    // Create 16 bits values from 8 bits data
    // Magnetometer
    mx=((Mag[1]<<8 | Mag[0]));
    my=((Mag[3]<<8 | Mag[2]));
    mz=((Mag[5]<<8 | Mag[4]));
}

int getHeading(){
    // Read magnetometer data  
    int16_t mx=0;
    int16_t my=0;
    int16_t mz=0;
    readMag(mx, my, mz);
    mx=(mx-mag_bias_x)*mag_SI_x;
    my=(my-mag_bias_y)*mag_SI_y;
    mz=(mz-mag_bias_z)*mag_SI_x;
    float heading = atan2(mx, my);
    
    //we're 12 degrees east, which is .209 radians
    float declinationAngle = .209;
    heading -= declinationAngle;
    
      // Correct for when signs are reversed.
    if(heading < 0) {
      heading += 2*PI;
    }
  
    // Check for wrap due to addition of declination.
    if(heading > 2*PI){
      heading -= 2*PI;
    }
  
    return (heading * 180/PI); 
  
}

void callibrateMag(){
    Serial.println("Callibrating...");
    int count = 0;
    int16_t x_max = -32766;
    int16_t x_min = 32766;
    int16_t y_max = -32766;
    int16_t y_min = 32766;
    int16_t z_max = -32766;
    int16_t z_min = 32766;
    int16_t mx;
    int16_t my;
    int16_t mz;
    while (count < 1000) {
      Serial.print(count);
      Serial.print(' ');
      readMag(mx, my, mz);
      if( mx > x_max){
        x_max = mx;
      }
      if( mx < x_min){
        x_min = mx;
      }
      if( my > y_max){
        y_max = my;
      }
      if( my < y_min){
        y_min = my;
      }
      if( mz > z_max){
        z_max = mz;
      }
      if( mz < z_min){
        z_min = mz;
      }
      count++;
    }
    // Hard iron callibration
    mag_bias_x = (x_max+x_min)/2;
    mag_bias_y = (y_max+y_min)/2;
    mag_bias_z = (z_max+z_min)/2;
    Serial.println();
    Serial.println("Done Callibrating...");
  
    // Soft iron callibration
    int mag_scale_x = (x_max+x_min)/2;
    int mag_scale_y = (y_max+y_min)/2;
    int mag_scale_z = (z_max+z_min)/2;
    float avg_rad = (mag_scale_x + mag_scale_y + mag_scale_z)/3;
    mag_SI_x = avg_rad/((float)mag_scale_x);
    mag_SI_y = avg_rad/((float)mag_scale_y);
    mag_SI_z = avg_rad/((float)mag_scale_z);
}


//
// Send Data //
//

void alignToAngle(int angle){
    // Align paperbot to angle with respect to magnetic north 
    int heading = getHeading();
    int divider = angle + 180;
    int dir = 20;
    bool flag1 = false;
    bool flag2 = false;
    if(divider > 360){
      divider -= 360;
      flag1 = true;
    }
    // Determine which direction to turn. Negative dir means left
    if(!flag1 && (heading > angle && heading < divider)){
        dir = -20;
    }
    else if(flag1 && (heading > angle || heading < divider)){
        dir = -20;
    }
    drive(90 + dir, 90 + dir);
    
    int angle_upper = angle + 6;
    int angle_lower = angle - 4;
    if(angle_upper > 360){
      angle_upper -= 360;
    }
    if(angle_lower < 0){
      angle_lower += 360;
    }
    //basically angle = 0
    if(angle_lower > angle_upper){
      flag2 = true;
    }
    while(true){
      heading = getHeading();
      if(!flag2 && ((heading < angle_upper) && (heading > angle_lower))){
        stop_s();
        break;
      }
      else if(flag2){ //could just align to 0, but might as well do math
         int temp_heading = heading + 360;
         int temp_upper = angle_upper + 360;
         if(((temp_heading < temp_upper) && (temp_heading > angle_lower)) || //heading > 0
         ((heading < temp_upper ) && (heading > angle_lower))){
            stop_s();
            break;
         }
      }
    }
}

void sendSensorData(uint8_t id){  
    int heading_degrees = getHeading(); 
    int angle = heading_degrees - box_a;
    alignToAngle(box_a);
    
    // Read range sensors
    int range_x = -2;
    int range_y = -2;
    readRange(range_x, range_y);

    
    // Finally, send data
    char tx[20] = "";
    sprintf(tx, "%d %d %d", angle, range_x, range_y);
    Serial.println(tx);
    wsSend(id, tx);
  
    delay(100);
    alignToAngle(heading_degrees);
    delay(100);
}

void smallTurn(int dir){
    //turn right
    if(dir == 0){
      drive(110, 110);
      delay(200);
      stop_s();
      delay(100);
    }
    else if(dir == 1){
      drive(70,70);
      delay(200);
      stop_s();
      delay(100);
    }
    
}

//get and send box dimensions
void getBoxDimensions(uint8_t id){
    DEBUG("Finding Dimensions of box");
    LED_ON;
    int heading_offset = getHeading();
    int x1 = -1;
    int y1 = -1;
    int x2 = -1;
    int y2 = -1;
    int counter1 = 0;
    int counter2 = 0;
    readRange(x1, y1);
    drive(110,110);
    while(true){
      int heading = getHeading();
      int range_x = -2;
      int range_y = -2;
      readRange(range_x, range_y);
      if(range_x < x1 + 5){
        x1 = range_x;
        y1 = range_y;
        heading_offset = heading;
        counter1++;
      }
      else if(range_x > (x1 + 10)){
        counter2++;
        if(counter1 > 10 && counter2 > 5){
          stop_s();
          break;
        }
      }
    }
    char tx[30] = "";
    sprintf(tx, "Finished 1, got %d %d %d", heading_offset, x1, y1);
    Serial.println(tx);
    wsSend(id, tx);
    delay(100);

    
    // now spin 180 degrees and get other side
    int temp = heading_offset + 180;
    if(temp > 360){
      temp -= 360;
    }
    alignToAngle(temp);
    readRange(x2, y2);
   
    sprintf(tx, "Finished 2, got %d %d", x2, y2);
    Serial.println(tx);
    wsSend(id, tx);
    delay(100);
    
    box_a = heading_offset;
    box_x = x1 + x2;
    box_y = y1 + y2;
    
    alignToAngle(box_a);
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if (payload[0] == '$') {
                int left_motor = 100*(payload[1]-'0') + 10*(payload[2]-'0') + payload[3]-'0';
                int right_motor = 100*(payload[5]-'0') + 10*(payload[6]-'0') + payload[7]-'0';
                drive(left_motor, right_motor); 
            }
            else if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Callibrating Magnetometer. Spin me please.");
                  delay(1000);
                  callibrateMag();
                  delay(1000);
                  wsSend(id, "Callibration Done.");
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                  delay(100);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                  delay(100);
                }
                else if(payload[1] == 'G'){
                  // get box dimensions
                  getBoxDimensions(id);

                  char tx[20] = "";
                  sprintf(tx, "%d %d %d", box_a, box_x, box_y);
                  Serial.println(tx);
                  wsSend(id, tx);
                  delay(100);
                }
                else if(payload[1] == 'T'){
                  smallTurn(0);
                  wsSend(id, "TestSmallRight");
                }
                else if(payload[1] == 'Z'){
                  int angle = ((payload[2] - '0')*100) + ((payload[3] - '0')*10) + ((payload[4] - '0'));
                  alignToAngle(angle);
                  int temp = getHeading();
                  char tx[20] = "";
                  sprintf(tx, "Aligning: %d", temp);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'S'){
                  // send sensor data
                  stop_s();
                  sendSensorData(id);
                  delay(100);
                }
                else{
                  stop_s();
                  delay(100);
                }
            }
            break;
    }
}
