#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include "hidjoystickrptparser.h"

#include <HardwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions odrive
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

//ODrive communication
HardwareSerial& odrive_serial2 = Serial2;
HardwareSerial& odrive_serial3 = Serial3;

// ODrive object
ODriveArduino odrive1(odrive_serial2);
ODriveArduino odrive2(odrive_serial3);

//Joystick derived
float theta = 0;
float v = 0;
float Vx = 0;
float Vy = 0;
float VA = 0;
float VB = 0;
float VC = 0;
float VD = 0;

//Logitech 310
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

float Buttons = JoyEvents.bu;
float lxa =  JoyEvents.lx;        //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
float lya = JoyEvents.ly;           //map(JoyEvents.ly, 0, 127, 0, 1023);                   // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);
float rxa = JoyEvents.rx;          //         map(JoyEvents.rx, 0, 127, 0, 1023); // map(JoyEvents.Z2, 0, 0xFF, 0.f, 255.f);
float rya = JoyEvents.ry;          // map(JoyEvents.ry, 0, 127, 0, 1023); // map(JoyEvents.Rz, 0, 0xFF, 0.f, 255.f);
//Group initialize
float blue = Joy.blue;
float green = Joy.green;
float red = Joy.red;
float yellow = Joy.yellow;
float L1 = Joy.lb;
float R1 = Joy.rb;
float gpad = JoyEvents.ht;
float L2 = Joy.lt;
float R2 = Joy.rt;
float back = Joy.bk;
float start = Joy.st;
float leftjoy = Joy.jl;
float rightjoy = Joy.jr;
 int result = 0;

void setup() {
  // put your setup code here, to run once:
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

  Serial.begin(115200);

  // ODrive uses 115200 baud
  odrive_serial2.begin(115200);
  odrive_serial3.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  lxa = lxa - 128;
  lya = lya - 127;
  rxa = rxa - 128;
  rya = rya - 127;

  Usb.Task();

  int maplx =  map(JoyEvents.lx, 0, 255, -100, 100);     //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f); JoyEvents.lx;
  int maply =  map(JoyEvents.ly, 0, 255, 100, -100);      // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);  JoyEvents.ly;


if(((-25 <= maplx) && (maplx <= 25)) && ((-25 <= maply) && (maply <= 25))){
v = 0; 
theta = 0; 

Serial.print(" *********************************** ");
theta = atan2(maply, maplx);
Serial.print(" \t theta : ");
Serial.print(theta*180/PI);
}
else {
  
  v = sqrt((pow(maplx,2)) + (pow(maply,2)));
Serial.print(" \t v : ");
Serial.print(v);

  theta = atan2(maply, maplx);
Serial.print(" \t theta : ");
Serial.print(theta*180/PI);
}

Serial.print(" \t maplx : ");
Serial.print(maplx);
Serial.print(" \t maply : ");
Serial.print(maply);

//  theta = atan2(maply, maplx);
//Serial.print(" \t theta : ");
//Serial.print(theta);

//  v = sqrt((pow(maplx,2)) + (pow(maply,2)));
//Serial.print(" \t v : ");
//Serial.print(v);

  Vx = v * cos(theta);
  Vy = v * sin(theta);
  
Serial.print(" \t Vx : ");
Serial.print(Vx);
Serial.print(" \t Vy : ");
Serial.print(Vy);

  VA = +(Vx * cos(0.785398)) - (Vy * sin(0.785398));
  VB = +(Vx * cos(0.785398)) + (Vy * sin(0.785398));
  VC = -(Vx * cos(0.785398)) - (Vy * sin(0.785398));
  VD = -(Vx * cos(0.785398)) + (Vy * sin(0.785398));

  int V4 =  map(VA, -100, 100, -1000, 1000);
  int V3 =  map(VB, -100, 100, -1000, 1000);
  int V2 =  map(VC, -100, 100, -1000, 1000);
  int V1 =  map(VD, -100, 100, -1000, 1000);

  Serial.print("\t V1 : ");
  Serial.print(V1);
  Serial.print("\t V2 : ");
  Serial.print(V2);
  Serial.print("\t V3 : ");
  Serial.print(V3);
  Serial.print("\t V4 : ");
  Serial.print(V4);

  Serial.print("\t result : ");
  Serial.println(result);


  int final_V1 = V1 + result;
  int final_V2 = V2 + result;
  int final_V3 = V3 + result;
  int final_V4 = V4 + result;

  odrive1.SetVelocity(0, final_V1);
  odrive1.SetVelocity(1, final_V2);
  odrive2.SetVelocity(0, final_V3);
  odrive2.SetVelocity(1, final_V4);

}
