package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.PublicTemperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.ReflectionUtils;

public class Robot extends LoggedRobot {

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        // ---------- Set Base Units ----------
        // Change Temperature Base Unit from Kelvin to Celsius.
        ReflectionUtils.modifyStaticFinalField(BaseUnits.class, "Temperature",
                new PublicTemperature(x -> x, x -> x, "Celsius", "°C"));
        ReflectionUtils.modifyStaticFinalField(Units.class, "Celsius",
                BaseUnits.Temperature);
        ReflectionUtils.modifyStaticFinalField(Units.class, "Kelvin",
                Units.derive(Units.Celsius).offset(-273.15).named("Kelvin").symbol("K").make());

        // Set up data receivers & replay source.
        switch (Constants.getCurrentOperatingMode()) {

            case REAL_WORLD:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIMULATION:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case LOG_REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;

            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        // Start AdvantageKit logger.
        Logger.start();

        // Log All Commands.
        CommandScheduler.getInstance().onCommandInitialize((command) -> {
            Logger.recordOutput("Commands", command.getName() + ":initialize");
        });
        CommandScheduler.getInstance().onCommandFinish((command) -> {
            Logger.recordOutput("Commands", command.getName() + ":finish");
        });

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // Unused.
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        // Unused.
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // TODO: Keep or discard depending on final Climber design.
        // if (Constants.hasClimberSubsystem()) {
        // CommandScheduler.getInstance().schedule(robotContainer.climber.setHeightCommand(Centimeters.of(1)));
        // }

        if (Constants.hasAimerSubsystem()) {
            CommandScheduler.getInstance()
                    .schedule(robotContainer.aimer.setAngleCommand(Rotation2d.fromDegrees(2)));
        }

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // Unused.
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Unused.
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        // Unused.
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        // Unused.
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        // Unused.
    }
}