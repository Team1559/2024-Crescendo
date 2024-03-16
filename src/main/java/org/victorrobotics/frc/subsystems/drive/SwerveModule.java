package org.victorrobotics.frc.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.encoder.EncoderIo;
import org.victorrobotics.frc.io.encoder.EncoderIoCtreCanCoder;
import org.victorrobotics.frc.io.encoder.EncoderIoInputsAutoLogged;
import org.victorrobotics.frc.io.encoder.EncoderIoReplay;
import org.victorrobotics.frc.io.encoder.EncoderIoSimulation;
import org.victorrobotics.frc.io.motor.MotorIo;
import org.victorrobotics.frc.io.motor.MotorIoInputsAutoLogged;
import org.victorrobotics.frc.io.motor.MotorIoReplay;
import org.victorrobotics.frc.io.motor.MotorIoSimulation;
import org.victorrobotics.frc.io.motor.talon_fx.MotorIoFalcon500;
import org.victorrobotics.frc.io.motor.talon_fx.MotorIoKrakenX60;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

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

    public final MotorIo driveMotor;
    private final MotorIoInputsAutoLogged driveMotorInputs = new MotorIoInputsAutoLogged();

    public final MotorIo steerMotor;
    private final MotorIoInputsAutoLogged steerMotorInputs = new MotorIoInputsAutoLogged();

    public final EncoderIo encoder;
    private final EncoderIoInputsAutoLogged encoderInputs = new EncoderIoInputsAutoLogged();

    private final String moduleName;

    private final PIDController drivePidController, steerPidController;
    private final SimpleMotorFeedforward driveFeedForwardController;

    private Rotation2d angleSetpoint; // Setpoint for closed loop control, null for open loop.
    private Double speedSetpoint; // Setpoint for closed loop control, null for open loop.

    private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute.

    public static SwerveModule createRealSwerveModule(WheelModuleIndex index) {
        return new SwerveModule(index,
                new MotorIoKrakenX60(Constants.getSwerveModuleHardwareIds().get(index).DRIVE_MOTOR_ID,
                        Constants.getCanivoreId(), true, NeutralModeValue.Brake, Rotation2d.fromRadians(0), null),
                new MotorIoFalcon500(Constants.getSwerveModuleHardwareIds().get(index).STEER_MOTOR_ID,
                        Constants.getCanivoreId(), false, NeutralModeValue.Brake, Rotation2d.fromRadians(0), null),
                new EncoderIoCtreCanCoder(Constants.getSwerveModuleHardwareIds().get(index).CANCODER_ID,
                        Constants.getCanivoreId(), false, Constants.getSwerveModuleEncoderOffsets().get(index)));
    }

    public static SwerveModule createReplaySwerveModule(WheelModuleIndex index) {
        return new SwerveModule(index, new MotorIoReplay(), new MotorIoReplay(), new EncoderIoReplay());
    }

    public static SwerveModule createSimulationSwerveModule(WheelModuleIndex index) {
        MotorIoSimulation mis;
        return new SwerveModule(index,
                new MotorIoSimulation(DCMotor.getKrakenX60(1), 6.75, MetersPerSecond.of(0.025)),
                mis = new MotorIoSimulation(DCMotor.getFalcon500(1), 150.0 / 7.0, MetersPerSecond.of(0.004)),
                new EncoderIoSimulation(mis));
    }

    private SwerveModule(WheelModuleIndex index, MotorIo driveMotor, MotorIo steerMotor, EncoderIo encoder) {

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.encoder = encoder;

        driveFeedForwardController = new SimpleMotorFeedforward(Constants.getDriveMotorPidValues().FF_S,
                Constants.getDriveMotorPidValues().FF_V);
        drivePidController = Constants.getDriveMotorPidValues().createController();
        steerPidController = Constants.getSteerMotorPidValues().createController();
        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        moduleName = "SwerveModule" + index.name();
    }

    public void periodic() {

        driveMotor.updateInputs(driveMotorInputs);
        Logger.processInputs(moduleName, driveMotorInputs);

        steerMotor.updateInputs(steerMotorInputs);
        Logger.processInputs(moduleName, steerMotorInputs);

        encoder.updateInputs(encoderInputs);
        Logger.processInputs(moduleName, encoderInputs);

        // On first cycle, reset relative turn encoder.
        // Wait until absolute angle is nonzero in case it wasn't initialized yet.
        if (turnRelativeOffset == null && encoderInputs.positionAbsolute.getRadians() != 0.0) {
            turnRelativeOffset = encoderInputs.positionAbsolute.minus(steerMotorInputs.positionAbsolute);
        }

        // Run closed loop turn control.
        if (angleSetpoint != null) {

            steerMotor
                    .setVoltage(Volts
                            .of(steerPidController.calculate(getAngle().getRadians(), angleSetpoint.getRadians())));

            // Run closed loop drive control.
            // Only allowed if closed loop turn control is running.
            if (speedSetpoint != null) {

                // Scale velocity based on turn error.
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(steerPidController.getPositionError());
                double velocityRadPerSec = adjustSpeedSetpoint / Constants.getWheelRadius().in(Meters);

                // Run drive controller.
                driveMotor.setVoltage(
                        Volts.of(driveFeedForwardController.calculate(velocityRadPerSec) + drivePidController
                                .calculate(driveMotorInputs.velocityActual.in(RadiansPerSecond), velocityRadPerSec)));
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
        steerMotor.setVoltage(Volts.zero());
        driveMotor.setVoltage(Volts.zero());

        // Disable closed loop control for turn and drive
        angleSetpoint = null;
        speedSetpoint = null;
    }

    /** @return The current turn angle of the module. */
    public Rotation2d getAngle() {
        return steerMotorInputs.positionAbsolute
                .plus(turnRelativeOffset == null ? Rotation2d.fromRotations(0) : turnRelativeOffset);
    }

    /** @return The current drive position of the module. */
    public Measure<Distance> getPosition() {
        return Meters.of(driveMotorInputs.positionAbsolute.getRotations() * Constants.getWheelRadius().in(Meters));
    }

    /** @return The current drive velocity of the module. */
    public Measure<Velocity<Distance>> getVelocity() {
        return MetersPerSecond
                .of(driveMotorInputs.velocityActual.in(RotationsPerSecond) * Constants.getWheelRadius().in(Meters));
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
        return driveMotor.getTemperature()
                .gt(driveMotor.getMaxSafeTemperature().times(Constants.getMotorSafeTemperatureBuffer()))
                || steerMotor.getTemperature()
                        .gt(steerMotor.getMaxSafeTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
    }
}