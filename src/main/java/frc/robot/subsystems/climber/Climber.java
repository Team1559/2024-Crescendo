package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    @AutoLog
    static class ClimberInputs { // TODO: Log everything that MotorIo is.

        public double currentRotationsLeft;
        public double currentRotationsRight;
        public double currentHeightAverageInInches;

        public double targetHeightInches;
        public double targetRotations;
    }

    // TODO: Move to Constants.
    private static final double ROTATIONS_PER_INCH = (5 * 5) * (5D / 3);

    // TODO: Use MotorIo variables instead.
    private final CANSparkMax motorL = new CANSparkMax(Constants.getClimberMotorIdLeft(), MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(Constants.getClimberMotorIdRight(), MotorType.kBrushless);

    private final PIDController controller = Constants.getAimerPid().createController();
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public Climber() {
        motorL.setInverted(true);
        motorR.setInverted(false);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));
        motorR.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));
        motorL.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
        motorL.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
    }

    @Override
    public void periodic() {

        // Log Inputs.
        updateInputs();
        Logger.processInputs("Climber/Climber", inputs);

        // Set Voltages.
        if (controller.getSetpoint() != 0) {
            motorL.setVoltage(controller.calculate(inputs.currentRotationsLeft));
            motorR.setVoltage(controller.calculate(inputs.currentRotationsRight));
        }
    }

    private void updateInputs() {

        inputs.currentRotationsLeft = motorL.getEncoder().getPosition();
        inputs.currentRotationsRight = motorR.getEncoder().getPosition();
        inputs.currentHeightAverageInInches = getHeight().in(Inches);

        inputs.targetRotations = controller.getSetpoint();
        inputs.targetHeightInches = inputs.targetRotations / ROTATIONS_PER_INCH;
    }

    // ========================= Functions =========================
    public void setHeight(Measure<Distance> height) {
        // TODO: Clamp the Voltage between Min/Max.
        controller.setSetpoint(height.in(Inches) * ROTATIONS_PER_INCH);
    }

    public void modifyHeight(Measure<Distance> change) {
        setHeight(getHeight().plus(change));
    }

    public Measure<Distance> getHeight() {
        return Inches
                .of(((motorL.getEncoder().getPosition() + motorR.getEncoder().getPosition()) / 2) / ROTATIONS_PER_INCH);
    }

    public Measure<Distance> getTargetHeight() {
        return Inches.of(controller.getSetpoint());
    }

    // TODO: Add Stop Method.

    // ========================= Commands =========================
    public Command setHeightCommand(Measure<Distance> height) {
        return new InstantCommand(() -> setHeight(height), this);
    }

    public Command modifyHeightCommand(Measure<Distance> change) {
        return new InstantCommand(() -> modifyHeight(change), this);
    }
}