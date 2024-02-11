
# WPILib Commands

Commands have 4 methods:
* `initialize` is called once when the command starts
* `execute` is called every robot clock cycle until the command is finished.
* `isFinished` is called every cycle and should return true when the command is finished. The default for commands is to return false, that is, to run forever.
* `done(boolean interrupted)` is called when the command is finished. The `interrupted` parameter will be true if the command was canceled before it finished.

## WPILib Command Reference

Class or Method                    |Description                                 
-----------------------------------|--------------------------------------------
<br>**Simple Commands**<br>        |                                            
InstantCommand                     |Accepts a function that becomes the `initialize` method.
RunCommand                         |Accepts a function that becomes the `execute` method.
StartEndCommand                    |Accepts two functions that become the `initialize` and `done` methods.
FunctionalCommand                  |Accepts four functions that become the four Command methods.
PrintCommand                       |Accepts a message and prints it once when the command is scheduled.
<br>**Command Groups**<br>         |                                            
SequentialCommandGroup             |Accepts multiple Commands and executes them in sequence.
ParallelCommandGroup               |Accepts multiple Commands and executes them in parallel.
ParallelRaceCommandGroup           |Accepts multiple Commands and executes them in parallel, until any one of them finishes. At that time the others are interrupted.
ParallelDeadlineCommandGroup       |Accepts multiple Commands and executes them in parallel, until the first one in the list (the deadline) finishes. At that time the others are interrupted.
<br>**Special Commands**<br>       |                                            
ConditionalCommand                 |Accepts two Commands and a boolean supplier to choose between them at runtime.
SelectCommand                      |Accepts a map of objects to commands, and an object supplier to choose among them at runtime.
RepeatCommand                      |Accepts a Command that it will run repeatedly, forever.
WaitCommand                        |Waits until the specified number of seconds has elapsed.
WaitUntilCommand                   |Accepts a boolean supplier, and waits until the supplier returns true.
ProxyCommand                       |Accepts a Command and schedules it separately from the current command group, ending when the command ends. The proxied command's requirements do not propagate to the containing group.
ScheduleCommand                    |Accepts multiple Command and schedules them separately from the current command group, then ends immediately.
<br>**Command methods**<br>        |<br>We generally don't use these; instead, instantiate one of the command classes directly.<br>
cmd.withTimeout                    |Returns a version of `cmd` that is interrupted after the specified number of seconds.
cmd.until or cmd.withInterrupt     |Returns a version of `cmd` that is interrupted when the boolean supplier returns true.
cmd.beforeStarting                 |Creates a SequentialCommandGroup that will run the provided Commands or function and then `cmd`.
cmd.andThen                        |Creates a SequentialCommandGroup that will run `cmd` and then the provided Commands or function.
cmd.alongWith                      |Creates a ParallelCommandGroup containing `cmd` and the supplied commands.
cmd.raceWith                       |Creates a ParallelRaceCommandGroup containing `cmd` and the supplied commands.
cmd.deadlineWith                   |Creates a ParallelDeadlineCommandGroup containing `cmd` and the supplied commands, where `cmd` is the deadline.
cmd.repeatedly                     |Returns a version of `cmd` that will be run repeatedly, forever.
cmd.asProxy                        |Returns `cmd` wrapped in a ProxyCommand.    
cmd.unless                         |Returns a version of `cmd` that will not run if provided boolean supplier returns true.
cmd.withInterruptBehavior          |Returns a version of `cmd` with the specified interruption behavior. Call with `kCancelIncoming` to make this command non-interruptible.
cmd.finallyDo                      |Returns a version of `cmd` that runs the provided function when the command ends. The function will receive a boolean interrupted parameter, like the `end` method.
cmd.handleInterrupt                |Returns a version of this command that will call the provided function when the command is interrupted.
<br>**Commands static methods**<br>|<br>We generally don't use these; instead, instantiate one of the command classes directly.<br>
Commands.none                      |Do-nothing command.                         
Commands.runOnce                   |Equivalent to `new InstantCommand`.         
Commands.run                       |Equivalent to `new RunCommand`.             
Commands.startEnd                  |Equivalent to `new StartEndCommand`.        
Commands.runEnd                    |Accepts two functions that become the `execute` and `end` methods.
Commands.either                    |Equivalent to `new ConditionalCommand`.     
Commands.select                    |Equivalent to `new SelectCommand`.          
Commands.sequence                  |Equivalent to `new SequentialCommandGroup`. 
Commands.parallel                  |Equivalent to `new ParallelCommandGroup`.   
Commands.race                      |Equivalent to `new ParallelRaceGroup`.      
Commands.deadline                  |Equivalent to `new ParallelDeadlineGroup`.  
Commands.repeatingSequence         |Equivalent to `new RepeatCommand(new new SequentialCommandGroup(...))`.
