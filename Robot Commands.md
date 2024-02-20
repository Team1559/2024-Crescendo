
# Robot Commands

## Controllers
* **Controller_1:** Driver Controller
* **Controller_2:** Co-Pilot Controller

## Default Commands
These Commands will run, unless another command is called on the subsystem(s).

* **Run Intake**
    * The Intake and Feeder motors will run, until the Color Sensor detects a Note
* **LEDs: Team Color**
    * The LEDs will display the team color
* **Stop Flywheel**
    * The Flywheel will be stopped, if it has not been given a command in over 2 seconds
        * (For safety)

## Triggers
These are Commands that are triggered by things other then the controllers.

* **LEDs: Has Note**
    * While the Color Sensor detects a Note, the LEDs will be $\color{orange}Orange$
* **Thermal Cutoffs**
    * Any Motor exceeding their recommended temperature range, will be disabled until its temperature returns to a safe range
        * (For safety)
    * LEDs will flash $\color{yellow}Yellow$ while the motor is disabled

# Controller Commands
These Commands are triggered by the controllers.

## Driving Commands
* **Drive Forward / Backwards**
* **Drive Left / Right**
* **Spin Clockwise / Counterclockwise**
* **Aim at Speaker**
    * The robot will maintain an aim at the Speaker
        * (This will disable the Spin Command)
    * The Flywheel will spin up
    * When in range, the LEDs will turn $\color{green}Green$
* **Aim at Amp**
    * The robot will maintain an aim at the Amp
        * (This will disable the Spin Command)
    * The Aimer will automatically set it's angle based on the distance to target
    * The Flywheel will spin up
    * When in range, the LEDs will turn $\color{green}Green$

## Intake Commands
Since the Intake operates automatically as a default command, these commands are only for abnormal situations.
* **Stop Intake & Feeder**
    * (The Flywheel will also be stopped, if running)
* **Reverse Intake & Feeder**
    * (The Flywheel will be stopped, if running)

## Shooter Commands
* **Shoot**
    * The Flywheel will spin up
        * (If not already spinning)
    * The Feeder will move the note to the Flywheel
    * LEDs will flash $\color{violet}Purple$
        * Once the Color Sensor no longer sees the Note
    * The Flywheel will stop
* **Stop Intake & Feeder**
    * (The Flywheel will also be stopped, if running)
* **Reverse Flywheel & Feeder**
    * (The Intake will be stopped, if running)

## Climber Commands
* **Climber Extend**
* **Climber Retract**
* **Traverse Left / Right**
