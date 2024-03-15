package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.io.encoder.EncoderIo;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.abstract_interface.DualMotorSubsystem;
import frc.robot.util.CommandUtils;
import frc.robot.util.MathUtils;

public class Aimer extends DualMotorSubsystem {

    @AutoLog
    static class AimerInputs {

        // ---------- Encoder ----------
        public Rotation2d targetAngle;
        public Rotation2d targetAngleClamped;
        public Rotation2d TargetAngleClampedVsCurrent;

        // ---------- Target ----------
        public boolean atTarget = false;
        public Measure<Distance> distanceToTarget;
    }

    // TODO: Wire Encoder to Motors.
    private final EncoderIo encoder;
    private final PIDController pidController;

    private final AimerInputsAutoLogged inputs = new AimerInputsAutoLogged();

    public Measure<Distance> distanceToTarget;
    private Rotation2d calculatedTargetAngle, targetAngle, targetAngleClamped;

    // ========================= Constructors ==================================

    public Aimer(MotorIo leftMotorIo, MotorIo rightMotorIo, EncoderIo encoderIo) {

        super(leftMotorIo, rightMotorIo);

        this.encoder = encoderIo;

        pidController = Constants.getAimerPid().createController();
    }

    // ========================= Periodic ======================================

    @Override
    public void periodic() {

        // Update Inputs.
        updateInputs();
        Logger.processInputs(getName(), inputs);

        // Set Voltages.
        if (targetAngleClamped != null) {

            double ff = Constants.getAimerPid().FF * targetAngleClamped.getCos();

            pidController.setSetpoint(targetAngleClamped.getDegrees());
            double output = pidController.calculate(encoder.getAbsolutePosition().getDegrees());

            double volts = MathUtil.clamp(ff + output, -1, 2);
            Measure<Voltage> voltage = Volts.of(volts);

            // TODO: What if motors are not moving evenly?
            setVoltage(voltage);
        }
    }

    private void updateInputs() {

        // ---------- Encoder ----------
        inputs.targetAngle = targetAngle;
        inputs.targetAngleClamped = targetAngleClamped;
        inputs.TargetAngleClampedVsCurrent = targetAngleClamped.minus(encoder.getAbsolutePosition());

        // ---------- Motor ----------
        inputs.atTarget = isAtTarget();
        inputs.distanceToTarget = distanceToTarget;
    }

    // ========================= Functions =====================================

    // -------------------- Actions --------------------

    public void aimAtTarget(Translation3d target, Translation2d currentPosition) {

        double distanceToTargetInMeters = currentPosition.getDistance(target.toTranslation2d());
        distanceToTarget = Meters.of(distanceToTargetInMeters);

        calculatedTargetAngle = Rotation2d.fromDegrees(
                1.42 * distanceToTargetInMeters * distanceToTargetInMeters - 15.8 * distanceToTargetInMeters + 55.3);

        setAnglePrivate(calculatedTargetAngle);
    }

    // -------------------- Getters --------------------

    /**
     * @return {@code true} if within error margin of target.
     */
    public boolean isAtTarget() {
        return Math.abs(targetAngleClamped.minus(encoder.getAbsolutePosition()).getDegrees()) <= Math
                .abs(Constants.getAimerErrorThreshold().getDegrees());
    }

    // -------------------- Modifiers --------------------

    public void modifyAngle(Rotation2d change) {
        if (targetAngle == null) {
            setAngle(change);
        } else {
            setAngle(encoder.getAbsolutePosition().plus(change));
        }
    }

    public void modifyTargetAngle(Rotation2d change) {
        if (targetAngle == null) {
            setAngle(change);
        } else {
            setAngle(targetAngle.plus(change));
        }
    }

    // -------------------- Setters --------------------

    public void setAngle(Rotation2d angle) {

        distanceToTarget = null;
        calculatedTargetAngle = null;

        setAnglePrivate(angle);
    }

    private void setAnglePrivate(Rotation2d angle) {
        targetAngle = angle;
        targetAngleClamped = MathUtils.clamp(angle, Constants.getAimerAngleRange().get_0(),
                Constants.getAimerAngleRange().get_1());
    }

    // ========================= Commands ======================================

    public Command aimAtTargetCommand(Supplier<Translation3d> target, Supplier<Translation2d> currentPosition) {
        return CommandUtils.addName(getName(),
                new InstantCommand(() -> this.aimAtTarget(target.get(), currentPosition.get()), this));
    }

    public Command modifyAngleCommand(Rotation2d change) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyAngle(change), this));
    }

    public Command modifyTargetAngleCommand(Rotation2d change) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyTargetAngle(change), this));
    }

    public Command setAngleCommand(Rotation2d angle) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setAngle(angle), this));
    }

    public Command waitUntilAtTargetCommand() {
        return CommandUtils.addName(getName(), new WaitUntilCommand(this::isAtTarget));
    }
}