#### Back to [README](/README.md)

# WPILib Commands

## Command Structure

Commands have 4 methods:
* `initialize` is called once when the command starts
* `execute` is called every robot clock cycle until the command is finished.
* `isFinished` is called every cycle and should return true when the command is finished. The default for commands is to return false, that is, to run forever.
* `done(boolean interrupted)` is called when the command is finished. The `interrupted` parameter will be true if the command was canceled before it finished.

## Command Behavior
### Instances
Command instances cannot be shared between Command Groups and/or Scheduling Commands.

### Running
Once a command is running, it will only stop if:
* The `isFinished` method returns `true`
    * This will cause `done(false)` to be called
* The `cancel` function is called on the command
    * This will cause `done(true)` to be called, regardless of the command's set [InterruptionBehavior](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.InterruptionBehavior.html)
    * _Note:_ `cancel` is called when a [Trigger](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html)'s `whileTrue` method ends
* Another command is scheduled that uses one of the same requirements, and the currently running command's set [InterruptionBehavior](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.InterruptionBehavior.html) is `kCancelSelf` _(default behavior)_
    * This will cause `done(true)` to be called

## Command Classes

Class|Description                                 
-----|-----------
<br/>**Simple Commands**                                                                                                                    |                                            
[InstantCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/InstantCommand.html)                  |Accepts a function that becomes the `initialize` method. (The "`isFinished`" method returns `true`.)
[RunCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/RunCommand.html)                          |Accepts a function that becomes the `execute` method. (The "`isFinished`" method returns `false`.)
[StartEndCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/StartEndCommand.html)                |Accepts two functions that become the `initialize` and `done` methods. (The "`isFinished`" method returns `false`.)
[FunctionalCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/FunctionalCommand.html)            |Accepts four functions that become the four Command methods.
<br/>**Command Groups**                                                                                                                     |<br/>(_Note: If a command group is interrupted, not all of the commands in the group are guaranteed to complete._)
[SequentialCommandGroup](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SequentialCommandGroup.html)  |Accepts multiple Commands and executes them in sequence.
[ParallelCommandGroup](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/ParallelCommandGroup.html)      |Accepts multiple Commands and executes them in parallel. The command group ends, when all commands in the group complete.
[ParallelRaceGroup](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/ParallelRaceGroup.html)            |Accepts multiple Commands and executes them in parallel, until any one of them finishes. At that time the others are interrupted.
[ParallelDeadlineGroup](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/ParallelDeadlineGroup.html)    |Accepts multiple Commands and executes them in parallel, until the first one in the list (the deadline) finishes. At that time the others are interrupted.
<br/>**Scheduling Commands**                                                                                                                |<br/>_These command schedule given commands._ [Note: The scheduled command(s) do not inherit this scheduling command's requirement(s).]
[DeferredCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/DeferredCommand.html)                |Accepts a Command Supplier that it will call on initialize.
[ConditionalCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/ConditionalCommand.html)          |Accepts two Commands and a boolean supplier to choose between them at runtime.
[SelectCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SelectCommand.html)                    |Accepts a map of Commands and an supplier that provides the map key to the command to choose at runtime.
[RepeatCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/RepeatCommand.html)                    |Accepts a Command that it will run and continuously restart, after it has ended, until the Repeat Command is interrupted.
[ProxyCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/ProxyCommand.html)                      |Accepts a Command or a Command Supplier and schedules it.
[ScheduleCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/ScheduleCommand.html)                |Accepts multiple Commands and schedules them separately from the current command, then ends immediately.
[NamedCommands](https://pathplanner.dev/api/java/com/pathplanner/lib/auto/NamedCommands.html)                                               |NamedCommands is a PathPlanner class that will register Names with a unique name for use by Path Planner. See: <https://pathplanner.dev/pplib-named-commands.html>
<br/>**Special Commands**                                                                                                                   |
[PrintCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/PrintCommand.html)                      |Is a special `InstantCommand` that accepts a message and prints it once when the command is scheduled.
[WaitCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/WaitCommand.html)                        |Waits until the specified number of seconds has elapsed.
[WaitUntilCommand](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/WaitUntilCommand.html)              |Has two use cases: <ul><li>Wait until the Match Timer is >= given seconds</li><li>Wait for a given BooleanSupplier to return true</li></ul>

## Command Methods
We generally don't use these; instead, instantiate one of the command classes directly, as this makes reading the code more clear and learning the command types easier.

[Command](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html) Methods|Description
--------------------------------------------------------------------------------------------------------------------|-----------
withTimeout             |Returns a version of Command that is interrupted after the specified number of seconds.
until or withInterrupt  |Returns a version of Command that is interrupted when the boolean supplier returns true.
beforeStarting          |Creates a SequentialCommandGroup that will run the provided Commands or function and then Command.
andThen                 |Creates a SequentialCommandGroup that will run Command and then the provided Commands or function.
alongWith               |Creates a ParallelCommandGroup containing Command and the supplied commands.
raceWith                |Creates a ParallelRaceCommandGroup containing Command and the supplied commands.
deadlineWith            |Creates a ParallelDeadlineCommandGroup containing Command and the supplied commands, where Command is the deadline.
repeatedly              |Returns a version of Command that will be run repeatedly, forever.
asProxy                 |Returns Command wrapped in a ProxyCommand.
unless                  |Returns a version of Command that will not run if provided boolean supplier returns true.
withInterruptBehavior   |Returns a version of Command with the specified interruption behavior. Call with `kCancelIncoming` to make this command non-interruptible.
finallyDo               |Returns a version of Command that runs the provided function when the command ends. The function will receive a boolean interrupted parameter, like the `end` method.
handleInterrupt         |Returns a version of this command that will call the provided function when the command is interrupted.

[Commands](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Commands.html) Methods|Preferred Equivalence
------|---------------------
deadline            |`ParallelDeadlineGroup`
defer               |`DeferredCommand`
deferredProxy       |`ProxyCommand`
either              |`ConditionalCommand`
idle                |`RunCommand`
none                |`RunCommand`
parallel            |`ParallelCommandGroup`
print               |`PrintCommand`
race                |`ParallelRaceGroup`
repeatingSequence   |`SequentialCommandGroup.repeatedly()`
run                 |`RunCommand`
runEnd              |`FunctionalCommand`
runOnce             |`RunCommand`
select              |`SelectCommand`
sequence            |`SequentialCommandGroup`
startEnd            |`StartEndCommand`
waitSeconds         |`WaitCommand`
waitUntil​           |`WaitUntilCommand`