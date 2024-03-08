package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Aimer extends SubsystemBase {

    @AutoLog
    static class AimerInputs {

        // ---------- Encoder ----------
        public double currentAngleInDegrees;
        public double targetAngleClampedInDegrees;
        public double targetAngleInDegrees;
        public double currentVsTargetAngleClampedInDegrees;

        // ---------- Motor ----------
        public boolean atTarget = false;

        public Measure<Current> lCurrentActual = Amps.zero();
        public Measure<Current> rCurrentActual = Amps.zero();

        public String[] lFaults = new String[0];
        public String[] rFaults = new String[0];

        public Rotation2d lPositionAbsolute = new Rotation2d();
        public Rotation2d rPositionAbsolute = new Rotation2d();

        /** From -1 to 1. */
        public float lPowerPercentage = 0;
        /** From -1 to 1. */
        public float rPowerPercentage = 0;

        public Measure<Temperature> lMotorTemp = Celsius.zero();
        public Measure<Temperature> rMotorTemp = Celsius.zero();

        public Measure<Velocity<Angle>> lVelocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> rVelocityActual = RotationsPerSecond.zero();

        public Measure<Voltage> lVoltsActual = Volts.zero();
        public Measure<Voltage> rVoltsActual = Volts.zero();

        public Measure<Voltage> lVoltsAvailable = Volts.zero();
        public Measure<Voltage> rVoltsAvailable = Volts.zero();

        public Measure<Voltage> voltsTarget;
    }

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.getAimerEncoderPort());

    // TODO: Update CANSparkMax variables as MotorIo variables.
    private final CANSparkMax motorL;
    private final CANSparkMax motorR;

    private final PIDController controller = Constants.getAimerPid().createController();

    private final boolean isLMotorInverted;

    private Rotation2d targetAngle, targetAngleClamped;
    private Measure<Voltage> targetVoltage;

    private final AimerInputsAutoLogged inputs = new AimerInputsAutoLogged();

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     **/
    public Aimer() {

        motorL = new CANSparkMax(Constants.getAimerMotorIdLeft(), MotorType.kBrushless);
        motorR = new CANSparkMax(Constants.getAimerMotorIdRight(), MotorType.kBrushless);

        // Randomly flips back. TODO: Figure out why?
        motorL.setInverted(false);
        motorR.setInverted(false);
        isLMotorInverted = false;

        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);

        motorL.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));
        motorR.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));

        motorL.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
        motorR.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
    }

    @Override
    public void periodic() {

        // Update Inputs.
        updateInputs();
        Logger.processInputs("Shooter/Aimer", inputs);

        // Set Voltages.
        if (controller.getSetpoint() != 0) {
            double ff = Constants.getAimerPid().FF * Rotation2d.fromDegrees(inputs.targetAngleInDegrees).getCos();
            double output = ff + controller.calculate(inputs.currentAngleInDegrees);
            output = MathUtil.clamp(output, -2, 2);
            targetVoltage = Volts.of(output);

            // TODO: What if motors are not moving evenly?
            motorL.setVoltage(isLMotorInverted ? -output : output);
            motorR.setVoltage(isLMotorInverted ? output : -output);
        }
    }

    private void updateInputs() {

        // ---------- Encoder ----------
        inputs.currentAngleInDegrees = getEncoderAbsolutePosition().getDegrees();
        inputs.targetAngleInDegrees = targetAngle.getDegrees();
        inputs.targetAngleClampedInDegrees = targetAngleClamped.getDegrees();
        inputs.currentVsTargetAngleClampedInDegrees = inputs.targetAngleClampedInDegrees - inputs.currentAngleInDegrees;

        // ---------- Motor ----------
        inputs.atTarget = atTarget();

        inputs.lCurrentActual = Amps.of(motorL.getOutputCurrent());
        inputs.rCurrentActual = Amps.of(motorR.getOutputCurrent());

        List<String> lFaults = new LinkedList<>();
        List<String> rFaults = new LinkedList<>();
        for (FaultID faultID : FaultID.values()) {
            if (motorL.getFault(faultID)) {
                lFaults.add(faultID.name());
            }
            if (motorR.getFault(faultID)) {
                rFaults.add(faultID.name());
            }
        }
        inputs.lFaults = lFaults.toArray(new String[0]);
        inputs.rFaults = rFaults.toArray(new String[0]);

        inputs.lPositionAbsolute = Rotation2d.fromRotations(motorL.getAbsoluteEncoder().getPosition());
        inputs.rPositionAbsolute = Rotation2d.fromRotations(motorR.getAbsoluteEncoder().getPosition());

        /** From -1 to 1. */
        inputs.lPowerPercentage = (float) motorL.getAppliedOutput();
        /** From -1 to 1. */
        inputs.rPowerPercentage = (float) motorR.getAppliedOutput();

        inputs.lMotorTemp = Units.Celsius.of(motorL.getMotorTemperature());
        inputs.rMotorTemp = Units.Celsius.of(motorR.getMotorTemperature());

        inputs.lVelocityActual = RevolutionsPerSecond.of(motorL.getEncoder().getVelocity());
        inputs.rVelocityActual = RevolutionsPerSecond.of(motorR.getEncoder().getVelocity());

        inputs.lVoltsActual = Volts.of(motorL.getBusVoltage() * motorL.getAppliedOutput());
        inputs.rVoltsActual = Volts.of(motorR.getBusVoltage() * motorR.getAppliedOutput());

        inputs.lVoltsAvailable = Volts.of(motorL.getBusVoltage());
        inputs.rVoltsAvailable = Volts.of(motorR.getBusVoltage());

        inputs.voltsTarget = targetVoltage;
    }

    // ========================= Functions =====================================

    public void aimAtTarget(Translation3d target, Translation2d currentPosition) {

        double distanceMeters = currentPosition.getDistance(target.toTranslation2d());
        Logger.recordOutput("Shooter/Aimer/DistanceToTarget", distanceMeters);

        Rotation2d angle = Rotation2d
                .fromDegrees(1.42 * distanceMeters * distanceMeters - 15.8 * distanceMeters + 55.3);
        Logger.recordOutput("Shooter/Aimer/CalculatedTargetAngleInDegrees", angle.getDegrees());

        setAngle(angle);
    }

    /**
     * @return {@code true} if within error margin of target.
     */
    public boolean atTarget() {
        return Math.abs(targetAngleClamped.minus(getEncoderAbsolutePosition()).getDegrees()) <= Math
                .abs(Constants.getAimerErrorThreshold().getDegrees());
    }

    public Rotation2d getEncoderAbsolutePosition() {
        // Invert angle as encoder is mounted "backwards".
        return Rotation2d.fromRotations(-encoder.getAbsolutePosition()).plus(Constants.getAimerEncoderOffset());
    }

    public void modifyAngle(Rotation2d change) {
        setAngle(getEncoderAbsolutePosition().plus(change));
    }

    public void modifyTargetAngle(Rotation2d change) {
        setAngle(targetAngle.plus(change));
    }

    public void setAngle(Rotation2d angle) {

        double clampedAngle = MathUtil.clamp(angle.getDegrees(), Constants.getAimerAngleRange().get_0().getDegrees(),
                Constants.getAimerAngleRange().get_1().getDegrees());
        controller.setSetpoint(clampedAngle);

        targetAngle = angle;
        targetAngleClamped = Rotation2d.fromDegrees(clampedAngle);
    }

    // ========================= Commands ======================================

    public Command aimAtTargetCommand(Supplier<Translation3d> target, Supplier<Translation2d> currentPosition) {
        return new InstantCommand(() -> this.aimAtTarget(target.get(), currentPosition.get()), this);
    }

    public Command modifyAngleCommand(Rotation2d change) {
        return new InstantCommand(() -> modifyAngle(change), this);
    }

    public Command modifyTargetAngleCommand(Rotation2d change) {
        return new InstantCommand(() -> modifyTargetAngle(change), this);
    }

    public Command setAngleCommand(Rotation2d angle) {
        return new InstantCommand(() -> setAngle(angle), this);
    }

    public Command waitUntilAtTargetCommand() {
        return new WaitUntilCommand(this::atTarget);
    }
}