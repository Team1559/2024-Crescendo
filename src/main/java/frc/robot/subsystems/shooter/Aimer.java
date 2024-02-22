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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aimer extends SubsystemBase {

    @AutoLog
    static class AimerInputs {

        public double currentAngleDegrees;
        public double targetAngleDegrees;

        public double lAppliedOutput, rAppliedOutput;
        public double lOutputCurrent, rOutputCurrent;
        public double lMotorTemp, rMotorTemp;
        public int lFaults, rFaults;
        public double lVelocity, rVelocity;
    }

    private final CANSparkMax motorL = new CANSparkMax(CONSTANTS.getAimerMotorIdLeft(), MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(CONSTANTS.getAimerMotorIdRight(), MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(CONSTANTS.getAimerEncoderPort());
    private final PIDController controller = new PIDController(CONSTANTS.getAimerPid().P, CONSTANTS.getAimerPid().I,
            CONSTANTS.getAimerPid().D);
    private final AimerInputsAutoLogged inputs = new AimerInputsAutoLogged();

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     **/
    public Aimer() {
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
        Logger.processInputs("Shooter/Aimer", inputs);

        // Set Voltages
        if (controller.getSetpoint() != 0) { // TODO: Determine Acceptable variance.
            double output = controller.calculate(inputs.currentAngleDegrees);
            motorL.setVoltage(output);
            motorR.setVoltage(output);
        }
    }

    private void updateInputs() {

        inputs.currentAngleDegrees = getAngle().getDegrees();
        inputs.targetAngleDegrees = getTargetAngle().getDegrees();

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
    }

    // ========================= Functions =====================================
    public void aimAtTarget(Translation3d target, Translation2d currentPosition) {

        double distance = currentPosition.getDistance(target.toTranslation2d());
        Rotation2d angle = new Rotation2d(distance, target.getZ());

        // Add Offset.
        angle = angle.minus(Rotation2d.fromDegrees(20)); // > 13 && < 30.

        // Flatten Slope.
        double COEFFICIENT = 1.00; // The Bigger the number (above 1), the flatter the slope.
        double angleInDegrees = Math.pow(angle.getDegrees(), 1 / COEFFICIENT);
        angle = Rotation2d.fromDegrees(angleInDegrees);

        Logger.recordOutput("Aimer/DistanceToTarget", distance);

        setTargetAngle(angle);
    }

    public void setTargetAngle(Rotation2d angle) {

        double targetAngle = MathUtil.clamp(angle.getDegrees(), CONSTANTS.getAimerAngleRange().get_0().getDegrees(),
                CONSTANTS.getAimerAngleRange().get_1().getDegrees());
        controller.setSetpoint(targetAngle);
    }

    public void modifyTargetAngle(Rotation2d change) {
        setTargetAngle(Rotation2d.fromDegrees(controller.getSetpoint()).plus(change));
    }

    public Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(controller.getSetpoint());
    }

    public Rotation2d getAngle() {
        // Invert angle as encoder is mounted "backwards".
        return Rotation2d.fromRotations(-encoder.getAbsolutePosition()).plus(CONSTANTS.getAimerEncoderOffset());
    }

    // ========================= Commands ======================================
    public Command setTargetAngleCommand(Rotation2d angle) {
        return new InstantCommand(() -> setTargetAngle(angle), this);
    }
}