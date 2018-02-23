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

int16_t x_max = -32766;
int16_t x_min = 32766;
int16_t y_max = -32766;
int16_t y_min = 32766;
int16_t z_max = -32766;
int16_t z_min = 32766;

// Callibration values. These values experimentally found for my room. 
int16_t mag_bias_x = 31;
int16_t mag_bias_y = 26;
int16_t mag_bias_z = 51;
float   mag_SI_x = 1.16;
float   mag_SI_y = 1.38;
float   mag_SI_z = 0.71;

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
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop_s();
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
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
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

void callibrateMag(){
  Serial.println("Callibrating...");
  int count = 0;
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
void send_sensor_data(uint8_t id){
  // Read magnetometer data  
  int16_t mx=0;
  int16_t my=0;
  int16_t mz=0;
  readMag(mx, my, mz);
  mx=(mx-mag_bias_x)*mag_SI_x;
  my=(my-mag_bias_y)*mag_SI_y;
  mz=(mz-mag_bias_z)*mag_SI_x;
  float heading = atan2(mx, my);

  //float declinationAngle = 0.209;
  //heading += declinationAngle;
  
    // Correct for when signs are reversed.
  if(heading < 0) {
    heading += 2*PI;
  }

  // Check for wrap due to addition of declination.
  if(heading > 2*PI){
    heading -= 2*PI;
  }

  float headingDegrees = heading * 180/PI; 
  char tx[20] = "";
  sprintf(tx, "%5.2f", headingDegrees);
  Serial.println(tx);
  wsSend(id, tx);

  delay(100);
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

            if (payload[0] == '#') {
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
                else {
                  // Everytime paperbot has to stop, send sensor data to update current state
                  stop_s();
                  send_sensor_data(id);
                  delay(100);
                }
                
            }
            break;
    }
}
