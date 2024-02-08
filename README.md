# Constants Class
Contains constants that need to be updated per robot. i.e.
* Capability Flags
* Hardware Ids
* Constraints and Tuning Parameters


# Tele-Op
* **Driving:** Left Joystick
  * D-Pad isolates directional commands for tuning
* **Spinning:** Triggers
* **LEDs:**
  * [Start + A] = Solid $\color{green}Green$
  * [Start + B] = Solid $\color{red}Red$ and Black pattern
  * [Start + X] = $\color{lightblue}Light \space Blue$ $\color{blue}Dark \space Blue$ pattern moving in one direction
  * [Start + Y] = $\color{yellow}Yellow$ and Black pattern moving in the other direction
  * [Left Bumber] = _Decrease Brightness_
  * [Right Bumber] = **Increase Brightness**
  * [Left + Right Bumber] = Turn Off LEDs

# Autonomous
* **TODO**

# Steps to Fork for New Season
## On or After January 1st
### Install Lastest Version of WPILib (VSCode) and Update Repo
* Download and Install the latest version of WPILib
  * See: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
  * When prompted select "Download for this computer only"
* Ensure that the following Extensions are installed:
  * [Extension Pack for Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack)
  * [Gradle Extension Pack](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-gradle-extension-pack)
* Ensure that the following Settings are set:
  * Editor: Format on Save
    * `checked`
  * Editor: Format on Save Mode
    * `file`
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

### Test the Robot
At this point everything should work. Test the robot and fix any issues. Push any fixed to the `main` branch.

## After the Game Announcement/Kickoff
* **TODO**