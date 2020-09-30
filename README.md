# mimirOpen

Open Hardware project outlining the hardware, software and design schematics for the sensor used by mimirHome.

## Table of Content

1. Design
1. Hardware
1. Software
1. Production
1. Resources

## Version 1.2

With the hardware for the most part all in the right place (except the battery monitor pin), it seems appropriate to move onto the next version. Here we will be optimizing the preformance of the battery and tiding up some of the functions so they're cleaner and more expressive for future adaptation.

In this version I focused on getting the core functionality of booting up the device, initializing systems and then putting the device to sleep. Depending on the bootCount of the device it would sometimes send data to the servers, or have light indications on the system status. Finally, I did some optimization of the timings of the device when it is on and asleep inorder to conserve battery life. As of 9/30 the device turns on for 25-30sec during bootup, sleeps for 5min and then turns on for 2.9sec when just reading and 9.8sec when sending data. This has resulted in the 2000mAh battery lasting a week or two.

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
