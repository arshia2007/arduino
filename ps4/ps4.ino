#include "USBHost_t36.h"

USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and communicate with USB Bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

void setup()
{
  Serial.begin(9600);  // Use standard Serial for Serial Monitor/Plotter
  myusb.begin();
  delay(2000);
  analogWriteResolution(14);
  analogWriteFrequency(0, 915527);
}

void loop()
{
  myusb.Task();     //Keep USB host tasks active

  if (joystick.available()) {
    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    int leftStickY = joystick.getAxis(1);
    int Lx = map(leftStickX, 0, 255, -8191, 8191);
    int Ly = map(leftStickY, 0, 255, 8191, -8191);

    // Right Stick values (axes 2 and 3)
    int rightStickX = joystick.getAxis(2);
    int x = map(rightStickX, 0, 16383, 8191, -8191);
    int rightStickY = joystick.getAxis(5);
    int y = map(rightStickY, 0, 16383, 8191, -8191);

    Serial.print("Lx: ");
    Serial.print(Lx);
    Serial.print("Ly: ");
    Serial.println(Ly); 

  }
}
