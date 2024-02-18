//Actual angle 5 deg. corresponds to reading 153 deg.
//Actual anlge 55 deg. correspods to reading 103 deg.
package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Aimer extends SubsystemBase {

    @AutoLog
    static class AimerInputs {

        public Rotation2d currentAngle;

        public double lAppliedOutput, rAppliedOutput;
        public double lOutputCurrent, rOutputCurrent;
        public double lMotorTemp, rMotorTemp;
        public int lFaults, rFaults;
        public double lVelocity, rVelocity;
        public Rotation2d targetAngle;
    }

    private final CANSparkMax motorL = new CANSparkMax(CONSTANTS.getAimerMotorIdLeft(), MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(CONSTANTS.getAimerMotorIdRight(), MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.AIMER_ENCODER_PORT);
    private final PIDController controller = new PIDController(Constants.AIMER_KP, Constants.AIMER_KI,
            Constants.AIMER_KD);
    private final AimerInputsAutoLogged inputs = new AimerInputsAutoLogged();

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     **/
    public Aimer() {
        motorL.setInverted(false);
        motorR.setInverted(true);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorR.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
    }

    @Override
    public void periodic() {
        // Log inputs
        updateInputs();
        Logger.processInputs("Shooter/Aimer", inputs);

        // Set Voltages
        if (controller.getSetpoint() != 0) {
            double output = controller.calculate(inputs.currentAngle.getDegrees());
            motorL.setVoltage(output);
            motorR.setVoltage(output);
        }
    }

    private void updateInputs() {

        inputs.currentAngle = getAngle();

        inputs.lAppliedOutput = motorL.getAppliedOutput();
        inputs.lOutputCurrent = motorL.getOutputCurrent();
        inputs.lMotorTemp = motorL.getMotorTemperature();
        inputs.lFaults = motorL.getFaults();
        inputs.lVelocity = motorL.getEncoder().getVelocity();

        inputs.rAppliedOutput = motorR.getAppliedOutput();
        inputs.rOutputCurrent = motorR.getOutputCurrent();
        inputs.rMotorTemp = motorR.getMotorTemperature();
        inputs.rFaults = motorR.getFaults();
        inputs.rVelocity = motorR.getEncoder().getVelocity();
        inputs.targetAngle = getTargetAngle();
    }

    // ========================= Functions =========================
    public void setTargetAngle(Rotation2d angle) {
        System.out.println("Set Angle: " + angle);
        double targetAngle = MathUtil.clamp(angle.getDegrees(), Constants.AIMER_LOWER_ANGLE,
                Constants.AIMER_UPPER_ANGLE);
        System.out.println(targetAngle);
        controller.setSetpoint(targetAngle);
    }

    public void modifyTargetAngle(Rotation2d change) {
        setTargetAngle(Rotation2d.fromDegrees(controller.getSetpoint()).plus(change));
    }

    public Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(controller.getSetpoint());
    }

    public Rotation2d getAngle() {
        // Neg. encoder position and add offset so that the angle is effectively
        // inverted
        return Rotation2d.fromRotations(-encoder.getAbsolutePosition()).plus(Constants.AIMER_ANGLE_OFFSET);
    }

    // ========================= Commands =========================
    public Command setTargetAngleCommand(Rotation2d angle) {
        return new InstantCommand(() -> setTargetAngle(angle), this);
    }
}