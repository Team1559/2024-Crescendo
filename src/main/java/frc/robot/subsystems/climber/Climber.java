package frc.robot.subsystems.climber;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    @AutoLog
    static class ClimberInputs { // TODO: Log everything that SingleMotorIo is.

        public double currentLeftPositionRotations;
        public double currentRightPositionRotations;
        public double currentAveragePositionRotations;
        // TODO: Add currentAverageHeightInches

        public double leftSetpointRotations;
        public double rightSetpointRotations;
    }

    private static final double ROTATIONS_PER_INCH = (5 * 5) * (5D / 3);

    private final CANSparkMax motorL = new CANSparkMax(CONSTANTS.getClimberMotorIdLeft(), MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(CONSTANTS.getClimberMotorIdRight(), MotorType.kBrushless);
    private final PIDController leftController = CONSTANTS.getClimberPid().createController();
    private final PIDController rightController = CONSTANTS.getClimberPid().createController();

    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public Climber() {
        motorL.setInverted(true);
        motorR.setInverted(false);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motorR.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motorL.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
        motorR.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
    }

    @Override
    public void periodic() {

        // Log Inputs.
        updateInputs();
        Logger.processInputs("Climber/Climber", inputs);

        // Set Voltages.
        if (leftController.getSetpoint() != 0) {
            double outputL = leftController.calculate(inputs.currentLeftPositionRotations);
            motorL.setVoltage(outputL);
        }
        if (rightController.getSetpoint() != 0) {
            double outputR = rightController.calculate(inputs.currentRightPositionRotations);
            motorR.setVoltage(outputR);
        }
    }

    private void updateInputs() {

        inputs.currentLeftPositionRotations = motorL.getEncoder().getPosition();
        inputs.currentRightPositionRotations = motorR.getEncoder().getPosition();
        inputs.currentAveragePositionRotations = (inputs.currentLeftPositionRotations
                + inputs.currentRightPositionRotations) / 2;

        inputs.leftSetpointRotations = leftController.getSetpoint();
        inputs.rightSetpointRotations = rightController.getSetpoint();
    }

    // ========================= Functions =========================
    // TODO: Take in Measure<Distance> instead.
    public void setTargetHeight(double height) {
        setTargetHeightLeft(height);
        setTargetHeightRight(height);
    }

    public void setTargetHeightLeft(double height) {
        double setpoint = height * ROTATIONS_PER_INCH;
        leftController.setSetpoint(setpoint);
    }

    public void setTargetHeightRight(double height) {
        double setpoint = height * ROTATIONS_PER_INCH;
        rightController.setSetpoint(setpoint);
    }

    // TODO: Take in Measure<Distance> instead.
    public void modifyTargetHeight(double changeHeightInches) {
        modifyTargetHeightLeft(changeHeightInches);
        modifyTargetHeightRight(changeHeightInches);
    }

    public void modifyTargetHeightLeft(double changeHeightInches) {
        setTargetHeightLeft(leftController.getSetpoint() / ROTATIONS_PER_INCH + changeHeightInches);
    }

    public void modifyTargetHeightRight(double changeHeightInches) {
        setTargetHeightRight(rightController.getSetpoint() / ROTATIONS_PER_INCH + changeHeightInches);
    }

    // ========================= Commands =========================
    // TODO: Take in Measure<Distance> instead.
    public Command setTargetHeightCommand(double heightInches) {
        return new InstantCommand(() -> setTargetHeight(heightInches), this);
    }

    public Command incrementTargetHeightCommand(double incrementInches) {
        return new InstantCommand(() -> modifyTargetHeight(incrementInches), this);
    }

    public Command incrementLeftHeightCommand(double increment) {
        return new InstantCommand(() -> modifyTargetHeightLeft(increment));
    }

    public Command incrementRightHeightCommand(double increment) {
        return new InstantCommand(() -> modifyTargetHeightRight(increment));
    }

    // TODO: Create all Commanp Functions.
}