# Constants Class
Contains constants that need to be updated per robot. i.e.
* Capability Flags
* Hardware Ids
* Constraints and Tuning Parameters

# Tele-Op
## Controller 1
This Controller is used by the Driver.
* **Driving:**
    * [Left Joystick]
* **Spinning:** Triggers
    * [Left Trigger] = Spin Counter-Clockwise
    * [Right Trigger] = Spin Clockwise
* **Shooting:** Letter Buttons
    * [Y + B] = Fire
    * [B] = Aim at speaker. Robot rotates to maintain aim; aimer tracks the target; flywheels run at speaker speed.
    * [B + Right Bumper] = Aim at amp. Robot rotates to maintain aim; aimer is at amp position; flywheels run at amp speed.
* **Emergency:**
    * [D-Pad Up] = Reverse Shooter

## Controller 2
This Controller is used by the Co-Pilot.

## Controller 3
This Controller is used by the Technician.
* **Driving:**
    * `[D-Pad]` Provides isolated directional commands
* **LEDs:** (On Hold)
    * `[A + Back]` = Solid $\color{green}Green$
    * `[B + Back]` = Solid $\color{red}Red$ and **Black** pattern
    * `[X + Back]` = $\color{blue}Blue$ and $\color{purple}Purple$ pattern moving in one direction
    * `[Y + Back]` = $\color{yellow}Yellow$ and **Black** pattern moving in the other direction
    * `[Left Bumber + Back]` = <span style="font-size:smaller;">Decrease Brightness</span>
    * `[Right Bumber + Back]` = <span style="font-size:larger;">Increase Brightness</span>
    * `[Back + Start]` = Turn Off LEDs
* **Subsystems:**
    * `[A]` Runs the Intake
    * `[A + Start]` Runs the Intake in Reverse
    * `[B]` Runs the Feeder
    * `[B + Start]` Runs the Feeder in Reverse
    * `[Y]` Runs the Flywheel
    * `[Y + Start]` Runs the Flywheel in Reverse
    * `[Left Bumper + Start]` Runs just the left Flywheel motor
    * `[Right Bumper + Start]` Runs just the left Flywheel motor
    * `[Left Bumper]` Moves the Aimer DOWN 5 degrees
    * `[Right Bumper]` Moves the Aimer UP 5 degrees

# Autonomous
During auto, the intake and flywheel run continuously to enable faster collecting and shooting of Notes.
See PathPlanner for the auto routes. We should print those out for use at competition.

Available auto routines:
* None: does nothing. Don't use this!
* DriveForward: drives 2m forward, scoring 2 points for leaving the starting zone. Starting position: 1m from the wall, 3.5m south of the speaker.
* DriveForwardAltPosition: same as DriveForward, but starts further north on the field. Starting position: 1m from the wall, 2.5m south of the speaker.
* ShootNoteAndDrive: Shoots a preloaded note, then drives out of the starting zone. Starting position: directly in front of the speaker.
* ShootPickShoot: Shoots a preloaded note, then drives forward to pick up the note and shoots it too. Starting position: directly in front of the speaker.
* All4: Shoots a preloaded note, then drives to pick up the note by the stage pillar, pulls up in front of the speaker to shoot, drives straight forward to pick up and shoot the note in front of the speaker, then picks up the other close note and shoots it.


# LED States

| Color  | State |
---------|--------
| Red    | Normal operation for Red alliance robot  |
| Blue   | Normal operation for Blue alliance robot |
| Orange | Note in the system; intake off.          |
| Violet | Single blink when shooting completes.    |
| Green  | Intake disabled by copilot.              |


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
  * Editor: Detect Indentation
    * `off`
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