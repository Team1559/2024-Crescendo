package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.io.swerve_module.SwerveModuleIo;
import frc.robot.io.swerve_module.SwerveModuleIoInputsAutoLogged;

public class SwerveModule {

    public enum WheelModuleIndex {
        /** 0 */
        FRONT_LEFT(0),
        /** 1 */
        FRONT_RIGHT(1),
        /** 2 */
        BACK_LEFT(2),
        /** 3 */
        BACK_RIGHT(3);

        public final int value;

        private WheelModuleIndex(int value) {
            this.value = value;
        }
    }

    private final SwerveModuleIo io;
    private final SwerveModuleIoInputsAutoLogged inputs = new SwerveModuleIoInputsAutoLogged();
    private final WheelModuleIndex index;

    private final SimpleMotorFeedforward driveFeedForward;

    private final PIDController driveFeedback;
    private final PIDController turnFeedback;

    private Rotation2d angleSetpoint; // Setpoint for closed loop control, null for open loop.
    private Double speedSetpoint; // Setpoint for closed loop control, null for open loop.

    private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute.

    public SwerveModule(SwerveModuleIo io, WheelModuleIndex index) {

        this.io = io;
        this.index = index;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.getCurrentOperatingMode()) {
            case REAL_WORLD:
            case LOG_REPLAY:
                // TODO: Move to Constants and Tune.
                driveFeedForward = new SimpleMotorFeedforward(0.1, 0.13);
                driveFeedback = new PIDController(0.05, 0.0, 0.0);
                turnFeedback = new PIDController(7.0, 0.0, 0.0);
                break;
            case SIMULATION:
                // TODO: Move to Constants and Tune.
                driveFeedForward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + index.name(), inputs);

        // On first cycle, reset relative turn encoder.
        // Wait until absolute angle is nonzero in case it wasn't initialized yet.
        if (turnRelativeOffset == null && inputs.cancoderOffsetPosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.cancoderOffsetPosition.minus(inputs.steerMotorPositionAbsolute);
        }

        // Run closed loop turn control.
        if (angleSetpoint != null) {

            io.setTurnVoltage(Volts.of(turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians())));

            // Run closed loop drive control.
            // Only allowed if closed loop turn control is running.
            if (speedSetpoint != null) {

                // Scale velocity based on turn error.
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());
                double velocityRadPerSec = adjustSpeedSetpoint / Constants.getWheelRadius().in(Meters);

                // Run drive controller.
                io.setDriveVoltage(Volts.of(driveFeedForward.calculate(velocityRadPerSec) +
                        driveFeedback.calculate(inputs.driveMotorVelocityActual.in(RadiansPerSecond),
                                velocityRadPerSec)));
            }
        }
    }

    // ========================= Functions =========================

    /**
     * Runs the module with the specified setpoint state, after optimizing it.
     * 
     * @return The optimized state.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {

        // Optimize state based on current angle.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Set Setpoints.
        angleSetpoint = optimizedState.angle;
        speedSetpoint = -optimizedState.speedMetersPerSecond; // TODO: Why is this negated?

        return optimizedState;
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(Volts.zero());
        io.setDriveVoltage(Volts.zero());

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
    }

    /** @return The current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.steerMotorPositionAbsolute
                .plus(turnRelativeOffset == null ? Rotation2d.fromRotations(0) : turnRelativeOffset);
    }

    /** @return The Temperature of the hottest motor. */
    public Measure<Temperature> getMaxTemperature() {
        return inputs.driveMotorTemp.gt(inputs.steerMotorTemp) ? inputs.driveMotorTemp : inputs.steerMotorTemp;
    }

    /** @return The current drive position of the module. */
    public Measure<Distance> getPosition() {
        return Meters.of(inputs.driveMotorPositionAbsolute.getRotations() * Constants.getWheelRadius().in(Meters));
    }

    /** @return The current drive velocity of the module. */
    public Measure<Velocity<Distance>> getVelocity() {
        return MetersPerSecond
                .of(inputs.driveMotorVelocityActual.in(RotationsPerSecond) * Constants.getWheelRadius().in(Meters));
    }

    /** @return The module position. */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    /** @return The module state (turn angle and drive velocity). */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public boolean isTemperatureTooHigh() {
        return getMaxTemperature().gt(io.getMaxSafeMotorTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
    }
}