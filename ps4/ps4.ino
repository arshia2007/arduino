#include "USBHost_t36.h"

USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

void setup()
{
  Serial1.begin(2000000);
  myusb.begin();
  delay(2000);
}

void loop()
{
  myusb.Task();     //Keep USB host tasks active

  if (joystick.available()) {
        // Left Stick values (axes 0 and 1)
        int leftStickX = joystick.getAxis(0);
        int leftStickY = joystick.getAxis(1);
        int Lx = map(leftStickX, 0, 255, -127, 127);
        int Ly = map(leftStickY, 0, 255, -127, 127);

        // Right Stick values (axes 2 and 3)
        int rightStickX = joystick.getAxis(2);
        int x = map(rightStickX, 0, 255, -127, 127);
        int rightStickY = joystick.getAxis(5);
        int y = map(rightStickY, 0, 255, -127, 127);

        // Display joystick values
        Serial.print("Left Stick X: ");
        Serial.print(Lx);
        Serial.print(" Left Stick Y: ");
        Serial.print(Ly);
        Serial.print(" | Right Stick X: ");
        Serial.print(x);
        Serial.print(" Right Stick Y: ");
        Serial.println(y);

  
    }
  }




