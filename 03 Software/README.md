# mimirOpen - Software
Software and firmware for the mimirOpen sensor.  The code is currently written in Platform IO so you can go into the source code and have a play around with it or just upload to a device.

## Setup with PlatformIO
1. Download or Fork the Repo to your local system.
1. Download Platform IO extension for VS Code
1. Open new window in VS Code, navigate to Platform IO Home
1. Click Projects Tab
1. Click "Add Existing"
1. Find the "03 Software" in the git repo and click "Open '03 Software'"

Now that you're project is setup you should be able to see additional tabs at the bottom of VS Code that will allow you to compile and upload the sketches found in the "src" folder to the device.

_If you open the main.cpp inside /src and VS Code is warning: `cannot open source file "Arduino.h"` then you will need to delete the .vscode folder, close out the window, open a new window and then open the project from Platform IO Home in the Projects tab._