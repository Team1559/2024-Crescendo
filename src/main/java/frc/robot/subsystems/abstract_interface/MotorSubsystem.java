package frc.robot.subsystems.abstract_interface;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.util.CommandUtils;

public interface MotorSubsystem extends Subsystem {

    // ========================= Static Helper Methods =========================
    public static String getSubsystemName(Class<? extends Subsystem> subsystemClass) {

        String packagePath = Optional.of(subsystemClass.getPackageName()).orElse("");
        String[] packageSegments = packagePath.split("\\.");

        String name = "";
        if (packageSegments.length > 0) {
            String lastPackageName = packageSegments[packageSegments.length - 1];
            name = lastPackageName + "/";
        }
        name += subsystemClass.getSimpleName();

        return name;
    }

    // ========================= Static Variables =================================

    public static List<MotorSubsystem> instantiatedSubsystems = Collections.synchronizedList(new LinkedList<>());

    // ========================= Configuration =================================

    /**
     * Restricts the motor speed both "forwards" and in "reverse". If the subsystem
     * it told to go faster, it will cap the speed to this maximum value.
     * 
     * @param maxVelocity The fastest (inclusive) the motor can go.
     */
    public void setMaxVelocity(Measure<Velocity<Angle>> maxVelocity);

    /**
     * Restricts the volts (both positive and negative) that will be sent to the
     * motor. If the subsystem it told to exceed this threshold, it will cap the
     * voltage to this maximum value.
     * 
     * @param maxVoltage The maximum voltage (inclusive) to send to the motor.
     */
    public void setMaxVoltage(Measure<Voltage> maxVoltage);

    /**
     * Restricts the motor movement. If the subsystem is told to move its position
     * outside this range, it will just go to the edge of the range.
     * 
     * @param minPosition The "lowest" / most "reverse" (inclusive) the motor can
     *                    go, or {@code null} if there is no limit.
     * @param maxPosition The "highest" / most "forward" (inclusive) the motor can
     *                    go, or {@code null} if there is no limit.
     */
    public void setMinAndMaxPositions(Rotation2d minPosition, Rotation2d maxPosition);

    // ========================= Periodic ======================================

    @Override
    public void periodic();

    // ========================= Functions =====================================

    // -------------------- Default Actions --------------------
    /**
     * Runs the motor in the "forwards" direction at the default forwards velocity
     * or voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void forward();

    /**
     * Runs the motor in the "forwards" direction at the maximum safe velocity of
     * the motor(s).
     */
    public void forwardMaxVelocity();

    /**
     * Runs the motor in the "reverse" direction at the default reverse velocity or
     * voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void reverse();

    /**
     * Runs the motor in the "reverse" direction at the maximum safe velocity of the
     * motor(s).
     */
    public void reverseMaxVelocity();

    /**
     * Stops the motor.
     */
    public void stop();

    // -------------------- Getters --------------------

    /**
     * @return The absolute position that the motor is currently at.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetPosition()}.
     *         </p>
     */
    public Rotation2d getCurrentAbsolutePosition();

    /**
     * @return The relative position that the motor is currently at.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetPosition()}.
     *         </p>
     */
    public Rotation2d getCurrentRelativePosition();

    /**
     * @return The speed that is motor is currently spinning at.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetVelocity()}.
     *         </p>
     */
    public Measure<Velocity<Angle>> getCurrentVelocity();

    /**
     * @return The voltage that is currently being set to the motor.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetVoltage()}.
     *         </p>
     */
    public Measure<Voltage> getCurrentVoltage();

    /**
     * @return The value that was last sent to {@link #setPosition(Rotation2d)}.
     */
    public Rotation2d getTargetPosition();

    /**
     * @return The value that was last sent to {@link #setVelocity(Measure)}.
     */
    public Measure<Velocity<Angle>> getTargetVelocity();

    /**
     * @return The value that was last sent to {@link #setVoltage(Measure)}.
     */
    public Measure<Voltage> getTargetVoltage();

    /**
     * @return {@code true}, if the motor is above the Maximum operating temperature
     *         minus the {@link Constants#getMotorSafeTemperatureBuffer()}.
     */
    public boolean isTemperatureTooHigh();

    // -------------------- Modifiers --------------------

    /**
     * Will change the current position by the given amount.
     * 
     * @param positionChange How much to change the current position by.
     */
    default void modifyPosition(Rotation2d positionChange) {
        setPosition(getCurrentAbsolutePosition().plus(positionChange));
    }

    /**
     * Will change the current speed by the given amount.
     * 
     * @param velocityChange How much to change the current speed by.
     */
    default void modifyVelocity(Measure<Velocity<Angle>> velocityChange) {
        setVelocity(getCurrentVelocity().plus(velocityChange));
    }

    /**
     * Will change the current voltage by the given amount.
     * 
     * @param voltageChange How many volts to change the current voltage by.
     */
    default void modifyVoltage(Measure<Voltage> voltageChange) {
        setVoltage(getCurrentVoltage().plus(voltageChange));
    }

    /**
     * Will change the target position by the given amount.
     * 
     * @param positionChange How much to change the current position by.
     */
    default void modifyTargetPosition(Rotation2d positionChange) {
        setPosition(getTargetPosition().plus(positionChange));
    }

    /**
     * Will change the target speed by the given amount.
     * 
     * @param velocityChange How much to change the current speed by.
     */
    default void modifyTargetVelocity(Measure<Velocity<Angle>> velocityChange) {
        setVelocity(getTargetVelocity().plus(velocityChange));
    }

    /**
     * Will change the target voltage by the given amount.
     * 
     * @param voltageChange How many volts to change the current voltage by.
     */
    default void modifyTargetVoltage(Measure<Voltage> voltageChange) {
        setVoltage(getTargetVoltage().plus(voltageChange));
    }

    // -------------------- Setters --------------------

    /**
     * Runs the motor(s) to a given position.
     * 
     * @param position The position to run the motor(s) to.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor(s) in the
     *                 "forwards" direction, and a negative value runs the motor(s)
     *                 in the "reverse" direction.
     *                 </p>
     */
    public void setPosition(Rotation2d position);

    /**
     * Runs the motor(s) at the given velocity.
     * 
     * @param velocity The speed at which to run the motor(s) at.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor(s) in the
     *                 "forwards" direction, and a negative value runs the motor(s)
     *                 in the "reverse" direction.
     *                 </p>
     */
    public void setVelocity(Measure<Velocity<Angle>> velocity);

    /**
     * Runs the motor(s) at the given voltage.
     * 
     * @param voltage How many volts to send to the motor(s).
     *                <p>
     *                <i>Note:</i> A positive value runs the motor(s) in the
     *                "forwards" direction, and a negative value runs the motor(s)
     *                in the "reverse" direction.
     *                </p>
     */
    public void setVoltage(Measure<Voltage> voltage);

    // ========================= Commands ======================================

    // -------------------- Default Actions --------------------

    /**
     * @return A {@link Command} that calls {@link #forward()} on
     *         {@link Command#initialize()}.
     */
    default Command forwardCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::forward, this));
    }

    /**
     * @return A {@link Command} that calls {@link #forward()} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command forwardThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::forward, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #forwardMaxVelocity()} on
     *         {@link Command#initialize()}.
     */
    default Command forwardMaxVelocityCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::forwardMaxVelocity, this));
    }

    /**
     * @return A {@link Command} that calls {@link #forwardMaxVelocity()} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command forwardMaxVelocityThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::forwardMaxVelocity, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #reverse()} on
     *         {@link Command#initialize()}.
     */
    default Command reverseCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::reverse, this));
    }

    /**
     * @return A {@link Command} that calls {@link #reverse()} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command reverseThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::reverse, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #reverse()} on
     *         {@link Command#initialize()}.
     */
    default Command reverseMaxVelocityCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::reverseMaxVelocity, this));
    }

    /**
     * @return A {@link Command} that calls {@link #reverse()} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command reverseMaxVelocityThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::reverseMaxVelocity, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #stop()} on
     *         {@link Command#initialize()}.
     */
    default Command stopCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::stop, this));
    }

    // -------------------- Modifiers --------------------

    /**
     * @return A {@link Command} that calls {@link #modifyPosition(Rotation2d)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyPositionCommand(Rotation2d positionChange) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyPosition(positionChange), this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyPosition(Rotation2d)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyPositionThenStopCommand(Rotation2d positionChange) {
        return CommandUtils.addName(getName(),
                new StartEndCommand(() -> modifyPosition(positionChange), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyPosition(Rotation2d)} on
     *         {@link Command#execute()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyPositionContinuouslyThenStopCommand(Rotation2d positionChange) {
        return CommandUtils.addName(getName(), new FunctionalCommand(
                () -> {
                },
                () -> modifyPosition(positionChange),
                (interrupted) -> this.stop(),
                () -> false,
                this));
    }

    /**
     * @return A {@link Command} that calls
     *         {@link #modifyTargetPosition(Rotation2d)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyTargetPositionCommand(Rotation2d positionChange) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyTargetPosition(positionChange), this));
    }

    /**
     * @return A {@link Command} that calls
     *         {@link #modifyTargetPosition(Rotation2d)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyTargetPositionThenStopCommand(Rotation2d positionChange) {
        return CommandUtils.addName(getName(),
                new StartEndCommand(() -> modifyTargetPosition(positionChange), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVelocity(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyVelocityCommand(Measure<Velocity<Angle>> velocityChange) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyVelocity(velocityChange), this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVelocity(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVelocityThenStopCommand(Measure<Velocity<Angle>> velocityChange) {
        return CommandUtils.addName(getName(),
                new StartEndCommand(() -> modifyVelocity(velocityChange), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVelocity(Measure)} on
     *         {@link Command#execute()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVelocityContinuouslyThenStopCommand(Measure<Velocity<Angle>> velocityChange) {
        return CommandUtils.addName(getName(), new FunctionalCommand(
                () -> {
                },
                () -> modifyVelocity(velocityChange),
                (interrupted) -> this.stop(),
                () -> false,
                this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVelocity(Measure)}
     *         on {@link Command#initialize()}.
     */
    default Command modifyTargetVelocityCommand(Measure<Velocity<Angle>> velocityChange) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyTargetVelocity(velocityChange), this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVelocity(Measure)}
     *         on {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyTargetVelocityThenStopCommand(Measure<Velocity<Angle>> velocityChange) {
        return CommandUtils.addName(getName(),
                new StartEndCommand(() -> modifyTargetVelocity(velocityChange), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVoltage(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyVoltageCommand(Measure<Voltage> voltageChange) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyVoltage(voltageChange), this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVoltage(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVoltageThenStopCommand(Measure<Voltage> voltageChange) {
        return CommandUtils.addName(getName(),
                new StartEndCommand(() -> modifyVoltage(voltageChange), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVoltage(Measure)} on
     *         {@link Command#execute()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVoltageContinuouslyThenStopCommand(Measure<Voltage> voltageChange) {
        return CommandUtils.addName(getName(), new FunctionalCommand(
                () -> {
                },
                () -> modifyVoltage(voltageChange),
                (interrupted) -> this.stop(),
                () -> false,
                this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVoltage(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyTargetVoltageCommand(Measure<Voltage> voltageChange) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> modifyTargetVoltage(voltageChange), this));
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVoltage(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyTargetVoltageThenStopCommand(Measure<Voltage> voltageChange) {
        return CommandUtils.addName(getName(),
                new StartEndCommand(() -> modifyTargetVoltage(voltageChange), this::stop, this));
    }

    // -------------------- Setters --------------------

    /**
     * @return A {@link Command} that calls {@link #setPosition(Rotation2d)} on
     *         {@link Command#initialize()}.
     */
    default Command setPositionCommand(Rotation2d position) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setPosition(position), this));
    }

    /**
     * @return A {@link Command} that calls {@link #setPosition(Rotation2d)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command setPositionThenStopCommand(Rotation2d position) {
        return CommandUtils.addName(getName(), new StartEndCommand(() -> setPosition(position), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #setVelocity(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command setVelocityCommand(Measure<Velocity<Angle>> velocity) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setVelocity(velocity), this));
    }

    /**
     * @return A {@link Command} that calls {@link #setVelocity(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command setVelocityThenStopCommand(Measure<Velocity<Angle>> velocity) {
        return CommandUtils.addName(getName(), new StartEndCommand(() -> setVelocity(velocity), this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #setVoltage(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command setVoltageCommand(Measure<Voltage> voltage) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setVoltage(voltage), this));
    }

    /**
     * @return A {@link Command} that calls {@link #setVoltage(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command setVoltageThenStopCommand(Measure<Voltage> voltage) {
        return CommandUtils.addName(getName(), new StartEndCommand(() -> setVoltage(voltage), this::stop, this));
    }

    // -------------------- Waits --------------------

    /**
     * @return A {@link Command} who's {@link Command#isFinished()} method returns
     *         {@code false} until {@link #isTemperatureTooHigh()} returns
     *         {@code false}.
     */
    default Command waitUntilTemperatureIsNotTooHighCommand() {
        Command command = new WaitUntilCommand(() -> !this.isTemperatureTooHigh());
        command.addRequirements(this);
        return CommandUtils.addName(getName(), command);
    }
}
