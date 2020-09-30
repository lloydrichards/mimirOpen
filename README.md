# mimirOpen

Open Hardware project outlining the hardware, software and design schematics for the sensor used by mimirHome.

## Table of Content

1. Design
1. Hardware
1. Software
1. Production
1. Resources

## Version 1.3

The purpose of version 1.3 is to get the device working properly and efficently on a battery with as long of a battery life as possible.  Hardware changes have been made to add in supply voltage supervisisors as well as correct mistakes with the switches and test different LDOs.  Finally an additional Fuel Gauge has been added for more accurate measurement of the battery life as well as for feedback while the device is charging or low battery.

## Software with PlatformIO

1. Download or Fork the Repo to your local system.
1. Download Platform IO extension for VS Code
1. Open new window in VS Code, navigate to Platform IO Home
1. Click Projects Tab
1. Click "Add Existing"
1. Find the "03 Software" in the git repo and click "Open '03 Software'"

Now that you're project is setup you should be able to see additional tabs at the bottom of VS Code that will allow you to compile and upload the sketches found in the "src" folder to the device.

_If you open the main.cpp inside /src and VS Code is warning: `cannot open source file "Arduino.h"` then you will need to delete the .vscode folder, close out the window, open a new window and then open the project from Platform IO Home in the Projects tab._

## BUGS

- Brownout on WiFi while on Battery
- Battery Monitoring on wrong Pin
- PNP for turning off LEDs while in deepsleep not working
- Power Switch not on EN of LDO
- Boot Button is redundent
