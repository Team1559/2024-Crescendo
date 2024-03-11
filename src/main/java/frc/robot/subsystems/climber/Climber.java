package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.io.motor.can_spark_max.MotorIoNeo550Brushless;
import frc.robot.subsystems.abstract_interface.DualMotorSubsystem;
import frc.robot.util.CommandUtils;
import frc.robot.util.MathUtils;

public class Climber extends DualMotorSubsystem {

    // ========================= Constructors ==================================

    public Climber() {
        super(
                new MotorIoNeo550Brushless(Constants.getClimberMotorIdLeft(), true, IdleMode.kBrake,
                        Rotation2d.fromRotations(0), Constants.getClimberPid()),
                new MotorIoNeo550Brushless(Constants.getClimberMotorIdRight(), false, IdleMode.kBrake,
                        Rotation2d.fromRotations(0), Constants.getClimberPid()));

        // TODO: Clamp height.
        // setMinAndMaxPositions(, );
    }

    // ========================= Functions =====================================

    // -------------------- Getters --------------------

    /** @return The average height of both motors. */
    public Measure<Distance> getCurrentHeight() {
        return MathUtils.average(getCurrentHeightLeft(), getCurrentHeightRight());
    }

    public Measure<Distance> getCurrentHeightLeft() {
        return Inches.of(getCurrentRelativePositionLeft().getRotations() / Constants.getClimberRotationsPerInch());
    }

    public Measure<Distance> getCurrentHeightRight() {
        return Inches.of(getCurrentRelativePositionRight().getRotations() / Constants.getClimberRotationsPerInch());
    }

    /** @return The average target hight of both motors. */
    public Measure<Distance> getTargetHeight() {
        return MathUtils.average(getTargetHeightLeft(), getTargetHeightRight());
    }

    public Measure<Distance> getTargetHeightLeft() {
        return Inches.of(getTargetPositionLeft().getRotations() / Constants.getClimberRotationsPerInch());
    }

    public Measure<Distance> getTargetHeightRight() {
        return Inches.of(getTargetPositionRight().getRotations() / Constants.getClimberRotationsPerInch());
    }

    // -------------------- Modifiers --------------------

    /** Changes the current height of both motors by the given amount. */
    public void modifyHeight(Measure<Distance> change) {
        setHeight(getCurrentHeight().plus(change));
    }

    /** Changes the current height of the left motor by the given amount. */
    public void modifyHeightLeft(Measure<Distance> change) {
        setHeightLeft(getCurrentHeightLeft().plus(change));
    }

    /** Changes the current height of the right motor by the given amount. */
    public void modifyHeightRight(Measure<Distance> change) {
        setHeightRight(getCurrentHeightRight().plus(change));
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

    // -------------------- Setters --------------------

    public void setHeight(Measure<Distance> height) {
        setHeightLeft(height);
        setHeightRight(height);
    }

    public void setHeightLeft(Measure<Distance> height) {
        setPositionLeft(Rotation2d.fromRotations(height.in(Inches) * Constants.getClimberRotationsPerInch()));
    }

    public void setHeightRight(Measure<Distance> height) {
        setPositionRight(Rotation2d.fromRotations(height.in(Inches) * Constants.getClimberRotationsPerInch()));
    }

    // ========================= Commands ======================================

    public Command modifyHeightCommand(Measure<Distance> change) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyHeight(change), this));
    }

    public Command modifyHeightLeftCommand(Measure<Distance> change) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyHeightLeft(change)));
    }

    public Command modifyHeightRightCommand(Measure<Distance> change) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyHeightRight(change)));
    }

    public Command setHeightCommand(Measure<Distance> height) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setHeight(height), this));
    }
}