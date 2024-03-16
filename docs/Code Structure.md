#### Back to [README](/README.md)

# Robot Code Structure

## Components

### Main
This is the starting point for the code. All it does is Create and Starts the Robot.

### Robot
* Creates Subsystems
* Creates Autonomous "Commands"
  * This includes our PathPlanner Routes
* Defines Default Commands
* Ties Commands to Triggers
* Ties Commands to Controllers
* Runs code on Initialize

### Subsystems
Defines Actions and Commands that can be ran on the Subsystem. Uses IO Clsses to interact with Hardware.

### IO Classes
Is the Hardware Abstraction layer.

### Compound Commands Class
Commands that use more that one Subsystem are defined here.

### Constants
Configurable fields are defined here, for ease of use.

## Diagram
![Diagram](imgs/Code%20Layout.svg)
_Google Docs Drawing: https://docs.google.com/drawings/d/1noqE-fVwvwOK8iTEaudckpXOD6EJBfzFJhOKPMGt7dg_