# ACTransmitter
A Transmitter which i made for playing Assetto Corsa and driving my RoboSoccer Robots (can be used with roboracers and sumobots too). it uses an esp32 to connect 2x joysticks and 4x buttons to the pc and send serial data to vjoyserialfeeder to emulate a dinput controller, and uses espnow for transmitting rc signals

# Requirements
Arduino IDE (with ESP32 Boards Installed) - https://www.arduino.cc/en/software/

Vjoy - https://github.com/shauleiz/vJoy/releases/tag/v2.1.8.39

VjoySerialFeeder - https://github.com/Cleric-K/vJoySerialFeeder

x360ce (for Xbox Controller Emulation) - https://github.com/x360ce/x360ce/releases/tag/3.2.9.82

# Installation
Install all the required software and open arduino ide -
  1. Install ESP32 Boards from Boards Manager
  2. Open the INO file from the ide provided in the release
  3. Select your ESP32 type and COM port on the top
  4. Upload the Sketch to the ESP32

Open "Configure Vjoy" and enable Device 1  

Open "VJoySerialFeeder" and add 4x Axis and 4x Buttons. Connect your VJoySerialFeeder to your ESP32 COM Port and remember to select VJoyDevice1.

//now you will have a working dinput controller but SOME GAMES DO NOT SUPPORT DINPUT! SOLUTION - 

Open X360ce (XBox emulator), add your Vjoy Controller in the software and map the axis and buttons to the xbox controller and start the emulator.

# ALL DONE!
