# Constants Class

Physcial things declared and tuning and ID's with firmware are all identified. 


# Tele-Op

Using the controller we made the joystick go forward, backward, left and right, along with being able to spin in place to the right or left. Using the D-Pad the robot can precisely move forward, backward and left and right. Everything is correct and well caliberatted. Right joystick is spin in place and left joystick is forward, backward, left and right. 

# Autonomous

The robot can move in a good sqaure, and it can be used for testing your callibrations. The name of this routine is Sqare Auto. Each side of the square is 1 meter.

# Steps to Fork for New Season
## On or After January 1st
### Update this repo to support the newly released version of WPILib
* Download and Install the latest version of WPILib
  * See: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
  * When prompted select "Download for this computer only"
* Open this repo in the latest version the WPILib VS Code
  * When prompted, migrated the code to support the lates vbersion of WPILib
* Update all Vendor Libraries
* Fix any Compile Errors
* Push changes to `main` branch

### Install Latest FRC Game Tools
* Follow these instructions to install the latest version of the Game Tools: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html
  * Make sure that you uninstall the old version first

### Update Robo Rio Image
* Follow these instructions to update the roboRIO Image: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/imaging-your-roborio.html

### Update Hardware Firmware
* If not already installed, download and install the Phoenix Tuner: https://store.ctr-electronics.com/software/
* Upen the Phoenix Tuner and connect it to the roboRIO
* Scan for everything on the CAN BUS
* Update the firmware of everything on the CAN BUS

### Test the Swirve Drive
At theis point everything should work. Test the robot and fix any issues. Push any fixed to the `main` branch.

## After the Game Announcement/Kickoff
* TODO