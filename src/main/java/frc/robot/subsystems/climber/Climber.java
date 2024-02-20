package frc.robot.subsystems.climber;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    @AutoLog
    static class ClimberInputs {
        public double currentLeftPositionRotations;
        public double currentRightPositionRotations;
        public double currentAveragePositionRotations;
        public double targetHeightInches;
        public double targetRotations;
    }

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
        // Log inputs
        updateInputs();
        Logger.processInputs("Climber/Climber", inputs);

        if (controller.getSetpoint() != 0) {
            double outputL = controller.calculate(inputs.currentLeftPositionRotations);
            double outputR = controller.calculate(inputs.currentRightPositionRotations);
            motorL.setVoltage(outputL);
            motorR.setVoltage(outputR);
        }
    }

    // 50 motor rotations = 1 inch vertical
    private void updateInputs() {

        inputs.currentLeftPositionRotations = motorL.getAbsoluteEncoder().getPosition();
        inputs.currentRightPositionRotations = motorL.getAbsoluteEncoder().getPosition();
        inputs.targetRotations = controller.getSetpoint();
        inputs.targetHeightInches = inputs.targetRotations / 50;
        inputs.currentAveragePositionRotations = (inputs.currentRightPositionRotations
                + inputs.currentLeftPositionRotations) / 2;
    }

    // ========================= Functions =========================
    public void setTargetHeight(double heightInches) {
        double targetHeight = MathUtil.clamp(heightInches, 0,
                CONSTANTS.getClimberMaxHeight());
        controller.setSetpoint(targetHeight * 50);
    }

    public void modifyTargetHeight(double changeHeightInches) {
        setTargetHeight(controller.getSetpoint() / 50 + changeHeightInches);
    }

    public double getCurrentTargetHeightInches() {
        return controller.getSetpoint() / 50;
    }

    // ========================= Commands =========================
    public Command setTargetHeightCommand(double heightInches) {
        return new InstantCommand(() -> setTargetHeight(heightInches), this);
    }
}