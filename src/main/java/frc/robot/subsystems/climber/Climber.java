package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    @AutoLog
    static class ClimberInputs { // TODO: Log everything that MotorIo is.

        public Measure<Current> currentActual = Amps.zero();

        public String[] faults = new String[0];

        public Measure<Distance> heightActual;
        public Measure<Distance> heightTarget;

        public Rotation2d positionAbsolute = new Rotation2d();
        public Rotation2d positionTarget = null;

        /** From -1 to 1. */
        public float powerPercentage = 0;

        public Measure<Temperature> temperature = Celsius.zero();

        public Measure<Velocity<Angle>> velocityActual = RotationsPerSecond.zero();

        public Measure<Voltage> voltsActual = Volts.zero();
        public Measure<Voltage> voltsAvailable = Volts.zero();
    }

    // TODO: Use MotorIo variables instead.
    private final CANSparkMax motorL;
    private final CANSparkMax motorR;

    private final boolean isLMotorInverted;

    Measure<Distance> targetHeightLeft, targetHeightRight;

    private final ClimberInputsAutoLogged motorLInputs = new ClimberInputsAutoLogged();
    private final ClimberInputsAutoLogged motorRInputs = new ClimberInputsAutoLogged();

    public Climber() {

        // Create & Configure Motor.
        motorL = new CANSparkMax(Constants.getClimberMotorIdLeft(), MotorType.kBrushless);
        motorR = new CANSparkMax(Constants.getClimberMotorIdRight(), MotorType.kBrushless);

        // Randomly flips back. TODO: Figure out why?
        motorL.setInverted(false);
        motorR.setInverted(false);
        isLMotorInverted = true;

        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);

        motorL.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));
        motorR.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));

        motorL.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
        motorL.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));

        // Configure piD Controller.
        motorL.getPIDController().setP(Constants.getAimerPid().P);
        motorL.getPIDController().setI(Constants.getAimerPid().I);
        motorL.getPIDController().setD(Constants.getAimerPid().D);
        motorL.getPIDController().setFF(Constants.getAimerPid().FF);

        motorR.getPIDController().setP(Constants.getAimerPid().P);
        motorR.getPIDController().setI(Constants.getAimerPid().I);
        motorR.getPIDController().setD(Constants.getAimerPid().D);
        motorR.getPIDController().setFF(Constants.getAimerPid().FF);
    }

    @Override
    public void periodic() {

        // Log Inputs.
        updateInputs(motorLInputs, motorL, targetHeightLeft);
        Logger.processInputs("Climber/Climber/LeftMotor", motorLInputs);

        updateInputs(motorRInputs, motorR, targetHeightRight);
        Logger.processInputs("Climber/Climber/RightMotor", motorRInputs);
    }

    private void updateInputs(ClimberInputsAutoLogged inputs, CANSparkMax motor, Measure<Distance> targetHeight) {

        inputs.powerPercentage = (float) motor.getAppliedOutput();

        inputs.currentActual = Amps.of(motor.getOutputCurrent());

        List<String> faults = new LinkedList<>();
        for (FaultID faultID : FaultID.values()) {
            if (motor.getFault(faultID)) {
                faults.add(faultID.name());
            }
        }
        inputs.faults = faults.toArray(new String[0]);

        inputs.heightActual = getHeight();
        inputs.heightTarget = targetHeight;

        inputs.positionAbsolute = Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition());
        inputs.positionTarget = Rotation2d
                .fromRotations(targetHeight.in(Inches) / Constants.getClimberRotationsPerInch());

        inputs.temperature = getTemperature();

        inputs.voltsActual = Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
        inputs.voltsAvailable = Volts.of(motor.getBusVoltage());

        inputs.velocityActual = RevolutionsPerSecond.of(motor.getEncoder().getVelocity());
    }

    // ========================= Functions =========================

    /** @return The average height of both motors. */
    public Measure<Distance> getHeight() {
        return Inches.of(getPosition().getRotations() / Constants.getClimberRotationsPerInch());
    }

    public Measure<Distance> getHeightLeft() {
        return Inches.of(getPositionLeft().getRotations() / Constants.getClimberRotationsPerInch());
    }

    public Measure<Distance> getHeightRight() {
        return Inches.of(getPositionRight().getRotations() / Constants.getClimberRotationsPerInch());
    }

    public Measure<Temperature> getMaxSafeTemperature() {
        // https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing
        return Celsius.of(40);
    }

    private Rotation2d getPosition() {
        return getPositionLeft().plus(getPositionRight()).div(2);
    }

    private Rotation2d getPositionLeft() {
        return Rotation2d.fromRotations(motorR.getEncoder().getPosition());
    }

    private Rotation2d getPositionRight() {
        return Rotation2d.fromRotations(motorL.getEncoder().getPosition());
    }

    /** @return The average target hight of both motors. */
    public Measure<Distance> getTargetHeight() {
        return targetHeightLeft.plus(targetHeightRight).divide(2);
    }

    public Measure<Distance> getTargetHeightLeft() {
        return targetHeightLeft;
    }

    public Measure<Distance> getTargetHeightRight() {
        return targetHeightRight;
    }

    /** @return The Temperature of the hottest motor. */
    public Measure<Temperature> getTemperature() {
        return Units.Celsius.of(Math.max(motorL.getMotorTemperature(), motorR.getMotorTemperature()));
    }

    public boolean isTemperatureTooHigh() {
        return getTemperature().gt(getMaxSafeTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
    }

    /** Changes the current height of both motors by the given amount. */
    public void modifyHeight(Measure<Distance> change) {
        setHeight(getHeight().plus(change));
    }

    /** Changes the current height of the left motor by the given amount. */
    public void modifyHeightLeft(Measure<Distance> change) {
        setHeightLeft(getHeightLeft().plus(change));
    }

    /** Changes the current height of the right motor by the given amount. */
    public void modifyHeightRight(Measure<Distance> change) {
        setHeightRight(getHeightRight().plus(change));
    }

    /** Changes the current height of both motors by the given amount. */
    public void modifyTargetHeight(Measure<Distance> change) {
        setHeight(getTargetHeight().plus(change));
    }

    /** Changes the current height of the left motor by the given amount. */
    public void modifyTargetHeightLeft(Measure<Distance> change) {
        setHeightLeft(getTargetHeightLeft().plus(change));
    }

    /** Changes the current height of the right motor by the given amount. */
    public void modifyTargetHeightRight(Measure<Distance> change) {
        setHeightRight(getTargetHeightRight().plus(change));
    }

    public void setHeight(Measure<Distance> height) {
        setHeightLeft(height);
        setHeightRight(height);
    }

    public void setHeightLeft(Measure<Distance> height) {
        targetHeightLeft = height;
        // TODO: Clamp height and log clamped height separatly.
        setPositionLeft(Rotation2d.fromRotations(height.in(Inches) * Constants.getClimberRotationsPerInch()));
    }

    public void setHeightRight(Measure<Distance> height) {
        targetHeightRight = height;
        // TODO: Clamp height and log clamped height separatly.
        setPositionRight(Rotation2d.fromRotations(height.in(Inches) * Constants.getClimberRotationsPerInch()));
    }

    private void setPositionLeft(Rotation2d position) {
        motorL.getPIDController().setReference(isLMotorInverted ? -position.getRotations() : position.getRotations(),
                CANSparkMax.ControlType.kPosition);
    }

    private void setPositionRight(Rotation2d position) {
        motorR.getPIDController().setReference(isLMotorInverted ? position.getRotations() : -position.getRotations(),
                CANSparkMax.ControlType.kPosition);
    }

    public void stop() {
        motorL.stopMotor();
        motorR.stopMotor();
        targetHeightLeft = targetHeightRight = null;
    }

    // ========================= Commands =========================

    public Command modifyHeightCommand(Measure<Distance> change) {
        return new InstantCommand(() -> modifyHeight(change), this);
    }

    public Command modifyTargetHeightLeftCommand(Measure<Distance> change) {
        return new InstantCommand(() -> modifyTargetHeightLeft(change));
    }

    public Command modifyTargetHeightRightCommand(Measure<Distance> change) {
        return new InstantCommand(() -> modifyTargetHeightRight(change));
    }

    public Command setHeightCommand(Measure<Distance> height) {
        return new InstantCommand(() -> setHeight(height), this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }
}