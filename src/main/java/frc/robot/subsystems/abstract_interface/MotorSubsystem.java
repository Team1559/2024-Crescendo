package frc.robot.subsystems.abstract_interface;

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

public interface MotorSubsystem extends Subsystem {

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
     * Runs the motor in the "reverse" direction at the default reverse velocity or
     * voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void reverse();

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
     * @return The voltage that is currently being set to the motor.
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
     * Runs the motor to a given position.
     * 
     * @param position The position to run the motor to.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor in the
     *                 "forwards" direction, and a negative value runs the motor in
     *                 the "reverse" direction.
     *                 </p>
     */
    public void setPosition(Rotation2d position);

    /**
     * Runs the motor at the given velocity.
     * 
     * @param velocity The speed at which to run the motor at.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor in the
     *                 "forwards" direction, and a negative value runs the motor in
     *                 the "reverse" direction.
     *                 </p>
     */
    public void setVelocity(Measure<Velocity<Angle>> velocity);

    /**
     * Runs the motor at the given voltage.
     * 
     * @param voltage How many volts to send to the motor.
     *                <p>
     *                <i>Note:</i> A positive value runs the motor in the "forwards"
     *                direction, and a negative value runs the motor in the
     *                "reverse" direction.
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
        return new InstantCommand(this::forward, this);
    }

    /**
     * @return A {@link Command} that calls {@link #forward()} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command forwardThenStopCommand() {
        return new StartEndCommand(this::forward, this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #reverse()} on
     *         {@link Command#initialize()}.
     */
    default Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }

    /**
     * @return A {@link Command} that calls {@link #reverse()} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command reverseThenStopCommand() {
        return new StartEndCommand(this::reverse, this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #stop()} on
     *         {@link Command#initialize()}.
     */
    default Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    // -------------------- Modifiers --------------------

    /**
     * @return A {@link Command} that calls {@link #modifyPosition(Rotation2d)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyPositionCommand(Rotation2d positionChange) {
        return new InstantCommand(() -> modifyPosition(positionChange), this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyPosition(Rotation2d)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyPositionThenStopCommand(Rotation2d positionChange) {
        return new StartEndCommand(() -> modifyPosition(positionChange), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyPosition(Rotation2d)} on
     *         {@link Command#execute()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyPositionContinuouslyThenStopCommand(Rotation2d positionChange) {
        return new FunctionalCommand(
                () -> {
                },
                () -> modifyPosition(positionChange),
                (interrupted) -> this.stop(),
                () -> false,
                this);
    }

    /**
     * @return A {@link Command} that calls
     *         {@link #modifyTargetPosition(Rotation2d)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyTargetPositionCommand(Rotation2d positionChange) {
        return new InstantCommand(() -> modifyTargetPosition(positionChange), this);
    }

    /**
     * @return A {@link Command} that calls
     *         {@link #modifyTargetPosition(Rotation2d)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyTargetPositionThenStopCommand(Rotation2d positionChange) {
        return new StartEndCommand(() -> modifyTargetPosition(positionChange), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVelocity(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyVelocityCommand(Measure<Velocity<Angle>> velocityChange) {
        return new InstantCommand(() -> modifyVelocity(velocityChange), this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVelocity(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVelocityThenStopCommand(Measure<Velocity<Angle>> velocityChange) {
        return new StartEndCommand(() -> modifyVelocity(velocityChange), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVelocity(Measure)} on
     *         {@link Command#execute()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVelocityContinuouslyThenStopCommand(Measure<Velocity<Angle>> velocityChange) {
        return new FunctionalCommand(
                () -> {
                },
                () -> modifyVelocity(velocityChange),
                (interrupted) -> this.stop(),
                () -> false,
                this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVelocity(Measure)}
     *         on {@link Command#initialize()}.
     */
    default Command modifyTargetVelocityCommand(Measure<Velocity<Angle>> velocityChange) {
        return new InstantCommand(() -> modifyTargetVelocity(velocityChange), this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVelocity(Measure)}
     *         on {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyTargetVelocityThenStopCommand(Measure<Velocity<Angle>> velocityChange) {
        return new StartEndCommand(() -> modifyTargetVelocity(velocityChange), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVoltage(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyVoltageCommand(Measure<Voltage> voltageChange) {
        return new InstantCommand(() -> modifyVoltage(voltageChange), this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVoltage(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVoltageThenStopCommand(Measure<Voltage> voltageChange) {
        return new StartEndCommand(() -> modifyVoltage(voltageChange), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyVoltage(Measure)} on
     *         {@link Command#execute()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyVoltageContinuouslyThenStopCommand(Measure<Voltage> voltageChange) {
        return new FunctionalCommand(
                () -> {
                },
                () -> modifyVoltage(voltageChange),
                (interrupted) -> this.stop(),
                () -> false,
                this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVoltage(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command modifyTargetVoltageCommand(Measure<Voltage> voltageChange) {
        return new InstantCommand(() -> modifyTargetVoltage(voltageChange), this);
    }

    /**
     * @return A {@link Command} that calls {@link #modifyTargetVoltage(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command modifyTargetVoltageThenStopCommand(Measure<Voltage> voltageChange) {
        return new StartEndCommand(() -> modifyTargetVoltage(voltageChange), this::stop, this);
    }

    // -------------------- Setters --------------------

    /**
     * @return A {@link Command} that calls {@link #setPosition(Rotation2d)} on
     *         {@link Command#initialize()}.
     */
    default Command setPositionCommand(Rotation2d position) {
        return new InstantCommand(() -> setPosition(position), this);
    }

    /**
     * @return A {@link Command} that calls {@link #setPosition(Rotation2d)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command setPositionThenStopCommand(Rotation2d position) {
        return new StartEndCommand(() -> setPosition(position), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #setVelocity(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command setVelocityCommand(Measure<Velocity<Angle>> velocity) {
        return new InstantCommand(() -> setVelocity(velocity), this);
    }

    /**
     * @return A {@link Command} that calls {@link #setVelocity(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command setVelocityThenStopCommand(Measure<Velocity<Angle>> velocity) {
        return new StartEndCommand(() -> setVelocity(velocity), this::stop, this);
    }

    /**
     * @return A {@link Command} that calls {@link #setVoltage(Measure)} on
     *         {@link Command#initialize()}.
     */
    default Command setVoltageCommand(Measure<Voltage> voltage) {
        return new InstantCommand(() -> setVoltage(voltage), this);
    }

    /**
     * @return A {@link Command} that calls {@link #setVoltage(Measure)} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    default Command setVoltageThenStopCommand(Measure<Voltage> voltage) {
        return new StartEndCommand(() -> setVoltage(voltage), this::stop, this);
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
        return command;
    }
}
