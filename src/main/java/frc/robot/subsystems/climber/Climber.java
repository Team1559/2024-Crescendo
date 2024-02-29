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

        public double targetHeightInches;
        public double targetRotations;
    }

    private static final double ROTATIONS_PER_INCH = (5 * 5) * (5D / 3);

    private final CANSparkMax motorL = new CANSparkMax(CONSTANTS.getClimberMotorIdLeft(), MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(CONSTANTS.getClimberMotorIdRight(), MotorType.kBrushless);
    private final PIDController controller = new PIDController(CONSTANTS.getAimerPid().P, CONSTANTS.getAimerPid().I,
            CONSTANTS.getAimerPid().D);

    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public Climber() {
        motorL.setInverted(false);
        motorR.setInverted(true);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motorR.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
        motorL.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
        motorL.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
    }

    @Override
    public void periodic() {

        // Log Inputs.
        updateInputs();
        Logger.processInputs("Climber/Climber", inputs);

        // Set Voltages.
        if (controller.getSetpoint() != 0) {
            double outputL = controller.calculate(inputs.currentLeftPositionRotations);
            double outputR = controller.calculate(inputs.currentRightPositionRotations);
            // TODO: Clamp the Voltage between Min/Max.
            motorL.setVoltage(outputL);
            motorR.setVoltage(outputR);
        }
    }

    private void updateInputs() {

        inputs.currentLeftPositionRotations = motorL.getEncoder().getPosition();
        inputs.currentRightPositionRotations = motorR.getEncoder().getPosition();
        inputs.currentAveragePositionRotations = (inputs.currentLeftPositionRotations
                + inputs.currentRightPositionRotations) / 2;

        inputs.targetRotations = controller.getSetpoint();
        inputs.targetHeightInches = inputs.targetRotations / ROTATIONS_PER_INCH;
    }

    // ========================= Functions =========================
    // TODO: Take in Measure<Distance> instead.
    public void setTargetHeight(double heightInches) {
        controller.setSetpoint(heightInches * ROTATIONS_PER_INCH);
    }

    // TODO: Take in Measure<Distance> instead.
    public void modifyTargetHeight(double changeHeightInches) {
        setTargetHeight(controller.getSetpoint() / ROTATIONS_PER_INCH + changeHeightInches);
    }

    // TODO: Return a Measure<Distance> instead.
    public double getCurrentTargetHeightInches() {
        return controller.getSetpoint() / ROTATIONS_PER_INCH;
    }

    // ========================= Commands =========================
    // TODO: Take in Measure<Distance> instead.
    public Command setTargetHeightCommand(double heightInches) {
        return new InstantCommand(() -> setTargetHeight(heightInches), this);
    }

    public Command incrementTargetHeightCommand(double incrementInches) {
        return new InstantCommand(() -> modifyTargetHeight(incrementInches), this);
    }

    // TODO: Create all Commanp Functions.
}