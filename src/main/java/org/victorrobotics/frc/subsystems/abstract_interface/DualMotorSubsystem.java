package org.victorrobotics.frc.subsystems.abstract_interface;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.motor.MotorIo;
import org.victorrobotics.frc.util.CommandUtils;
import org.victorrobotics.frc.util.MathUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DualMotorSubsystem extends SubsystemBase implements MotorSubsystem {

    @AutoLog
    static class DualMotorSubsystemInputs {

        public Measure<Velocity<Angle>> targetVelocityDifference, targetVelocityClampedDifference;
        public Measure<Voltage> targetVoltageDifference, targetVoltageClampedDifference;
        public Rotation2d targetPositionDifference, targetPositionClampedDifference;
    }

    public static List<DualMotorSubsystem> instantiatedSubsystems = Collections
            .synchronizedList(new LinkedList<>());

    protected final SingleMotorSubsystem leftMotor, rightMotor;

    private final DualMotorSubsystemInputsAutoLogged subsystemInputs = new DualMotorSubsystemInputsAutoLogged();

    // ========================= Constructors & Config =========================

    /**
     * Constricts a Dual Motor Subsystem.
     * 
     * @param leftMotorIo            The left Motor that belongs to this
     *                               {@link Subsystem}.
     *                               <p>
     *                               (<i>Note:</i> It should not be in any other
     *                               {@link Subsystem}.)
     *                               </p>
     * @param rightMotorIo           The right Motor that belongs to this
     *                               {@link Subsystem}.
     *                               <p>
     *                               (<i>Note:</i> It should not be in any other
     *                               {@link Subsystem}.)
     *                               </p>
     * @param defaultForwardVelocity The velocity that will be used by the
     *                               {@link #forward()} method.
     * @param defaultReverseVelocity The velocity that will be used by the
     *                               {@link #reverse()} method.
     * @param velocities             <i>Ignore.</i> (Used to allow constructor
     *                               overloading with generics.)
     */
    protected DualMotorSubsystem(MotorIo leftMotorIo, MotorIo rightMotorIo,
            Measure<Velocity<Angle>> defaultForwardVelocity, Measure<Velocity<Angle>> defaultReverseVelocity,
            Velocity<?>... velocities) {
        this.leftMotor = new SingleMotorSubsystem(leftMotorIo, defaultForwardVelocity, defaultReverseVelocity);
        this.rightMotor = new SingleMotorSubsystem(rightMotorIo, defaultForwardVelocity, defaultReverseVelocity);
        config();
    }

    /**
     * Constricts a Dual Motor Subsystem.
     * 
     * @param leftMotorIo           The left Motor that belongs to this
     *                              {@link Subsystem}.
     *                              <p>
     *                              (<i>Note:</i> It should not be in any other
     *                              {@link Subsystem}.)
     *                              </p>
     * @param rightMotorIo          The right Motor that belongs to this
     *                              {@link Subsystem}.
     *                              <p>
     *                              (<i>Note:</i> It should not be in any other
     *                              {@link Subsystem}.)
     *                              </p>
     * @param defaultForwardVoltage The voltage that will be used by the
     *                              {@link #forward()} method.
     * @param defaultReverseVoltage The voltage that will be used by the
     *                              {@link #reverse()} method.
     *                              or {@code null} if there is no limit.
     * @param velocities            <i>Ignore.</i> (Used to allow constructor
     *                              overloading with generics.)
     */
    protected DualMotorSubsystem(MotorIo leftMotorIo, MotorIo rightMotorIo, Measure<Voltage> defaultForwardVoltage,
            Measure<Voltage> defaultReverseVoltage, Voltage... voltages) {
        this.leftMotor = new SingleMotorSubsystem(leftMotorIo, defaultForwardVoltage, defaultReverseVoltage);
        this.rightMotor = new SingleMotorSubsystem(rightMotorIo, defaultForwardVoltage, defaultReverseVoltage);
        config();
    }

    /**
     * Constricts a Dual Motor Subsystem.
     * <p>
     * This means that the following methods will do nothing:
     * </p>
     * <ul>
     * <li>{@link #forward()}</li>
     * <li>{@link #forwardLeft()}</li>
     * <li>{@link #forwardRight()}</li>
     * <li>{@link #reverse()}</li>
     * <li>{@link #reverseLeft()}</li>
     * <li>{@link #reverseRight()}</li>
     * </ul>
     * 
     * @param leftMotorIo  The left Motor that belongs to this
     *                     {@link Subsystem}.
     *                     <p>
     *                     (<i>Note:</i> It should not be in any other
     *                     {@link Subsystem}.)
     *                     </p>
     * @param rightMotorIo The right Motor that belongs to this
     *                     {@link Subsystem}.
     *                     <p>
     *                     (<i>Note:</i> It should not be in any other
     *                     {@link Subsystem}.)
     *                     </p>
     */
    protected DualMotorSubsystem(MotorIo leftMotorIo, MotorIo rightMotorIo) {
        this.leftMotor = new SingleMotorSubsystem(leftMotorIo);
        this.rightMotor = new SingleMotorSubsystem(rightMotorIo);
        config();
    }

    private void config() {

        // Set Name.
        String name = MotorSubsystem.getSubsystemName(this.getClass());
        super.setName(name);

        leftMotor.setName(name + ":L");
        rightMotor.setName(name + ":R");

        // Add to Collection.
        instantiatedSubsystems.add(this);
        MotorSubsystem.instantiatedSubsystems.add(this);
    }

    @Override
    public void setMaxVelocity(Measure<Velocity<Angle>> maxVelocity) {
        leftMotor.setMaxVelocity(maxVelocity);
        rightMotor.setMaxVelocity(maxVelocity);
    }

    @Override
    public void setMaxVoltage(Measure<Voltage> maxVoltage) {
        leftMotor.setMaxVoltage(maxVoltage);
        rightMotor.setMaxVoltage(maxVoltage);
    }

    @Override
    public void setMinAndMaxPositions(Rotation2d minPosition, Rotation2d maxPosition) {
        leftMotor.setMinAndMaxPositions(minPosition, maxPosition);
        rightMotor.setMinAndMaxPositions(minPosition, maxPosition);
    }

    // ========================= Periodic ======================================

    @Override
    public void periodic() {

        // ---------- Log Subsystem Inputs ----------
        subsystemInputs.targetVelocityDifference = MathUtils.difference(leftMotor.subsystemInputs.targetVelocity,
                rightMotor.subsystemInputs.targetVelocity);
        subsystemInputs.targetVelocityClampedDifference = MathUtils.difference(
                leftMotor.subsystemInputs.targetVelocityClamped, rightMotor.subsystemInputs.targetVelocityClamped);

        subsystemInputs.targetVoltageDifference = MathUtils.difference(leftMotor.subsystemInputs.targetVoltage,
                rightMotor.subsystemInputs.targetVoltage);
        subsystemInputs.targetVoltageClampedDifference = MathUtils.difference(
                leftMotor.subsystemInputs.targetVoltageClamped, rightMotor.subsystemInputs.targetVoltageClamped);

        subsystemInputs.targetPositionDifference = MathUtils.difference(leftMotor.subsystemInputs.targetPosition,
                rightMotor.subsystemInputs.targetPosition);
        subsystemInputs.targetPositionClampedDifference = MathUtils.difference(
                leftMotor.subsystemInputs.targetPositionClamped, rightMotor.subsystemInputs.targetPositionClamped);

        Logger.processInputs(getName(), subsystemInputs);
    }

    // ========================= Functions =====================================

    // -------------------- Default Actions --------------------

    @Override
    public void forward() {
        leftMotor.forward();
        rightMotor.forward();
    }

    @Override
    public void forwardMaxVelocity() {
        leftMotor.forwardMaxVelocity();
        rightMotor.forwardMaxVelocity();
    }

    /**
     * Runs the left motor in the "forwards" direction at the default forwards
     * velocity or voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void forwardLeftMotor() {
        leftMotor.forward();
    }

    /**
     * Runs the right motor in the "forwards" direction at the default forwards
     * velocity or voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void forwardRightMotor() {
        rightMotor.forward();
    }

    @Override
    public void reverse() {
        leftMotor.reverse();
        rightMotor.reverse();
    }

    @Override
    public void reverseMaxVelocity() {
        leftMotor.reverseMaxVelocity();
        rightMotor.reverseMaxVelocity();
    }

    /**
     * Runs the left motor in the "reverse" direction at the default reverse
     * velocity or voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void reverseLeftMotor() {
        leftMotor.reverse();
    }

    /**
     * Runs the right motor in the "reverse" direction at the default reverse
     * velocity or
     * voltage.
     * <p>
     * <i>Note:</i> If no default Velocity or Voltage has been set, then the motor
     * will be stopped for safety.
     * </p>
     */
    public void reverseRightMotor() {
        rightMotor.reverse();
    }

    @Override
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    // -------------------- Getters --------------------

    /**
     * @return The average absolute position of both where both motors are currently
     *         at.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetPosition()}.
     *         </p>
     */
    @Override
    public Rotation2d getCurrentAbsolutePosition() {
        return MathUtils.average(leftMotor.getCurrentAbsolutePosition(), rightMotor.getCurrentAbsolutePosition());
    }

    /**
     * @return The absolute position of where the left motor currently is at.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetPositionLeft()}.
     *         </p>
     */
    public Rotation2d getCurrentAbsolutePositionLeft() {
        return leftMotor.getCurrentAbsolutePosition();
    }

    /**
     * @return The absolute position of where the right motor currently is at.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetPositionRight()}.
     *         </p>
     */
    public Rotation2d getCurrentAbsolutePositionRight() {
        return rightMotor.getCurrentAbsolutePosition();
    }

    /**
     * @return The average relative position of both where both motors are currently
     *         at.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetPosition()}.
     *         </p>
     */
    @Override
    public Rotation2d getCurrentRelativePosition() {
        return MathUtils.average(leftMotor.getCurrentRelativePosition(), rightMotor.getCurrentRelativePosition());
    }

    /**
     * @return The relative position of where the left motor currently is at.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetPositionLeft()}.
     *         </p>
     */
    public Rotation2d getCurrentRelativePositionLeft() {
        return leftMotor.getCurrentRelativePosition();
    }

    /**
     * @return The relative position of where the right motor currently is at.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetPositionRight()}.
     *         </p>
     */
    public Rotation2d getCurrentRelativePositionRight() {
        return rightMotor.getCurrentRelativePosition();
    }

    /**
     * @return The average speed that both motors are currently spinning at.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetVelocity()}.
     *         </p>
     */
    @Override
    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return MathUtils.average(leftMotor.getCurrentVelocity(), rightMotor.getCurrentVelocity());
    }

    /**
     * @return The speed that the left motor is currently spinning at.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetVelocityLeft()}.
     *         </p>
     */
    public Measure<Velocity<Angle>> getCurrentVelocityLeft() {
        return leftMotor.getCurrentVelocity();
    }

    /**
     * @return The speed that the right motor is currently spinning at.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetVelocityRight()}.
     *         </p>
     */
    public Measure<Velocity<Angle>> getCurrentVelocityRight() {
        return rightMotor.getCurrentVelocity();
    }

    /**
     * @return The average voltage that is currently being set to both motors.
     *         <p>
     *         <b>Note:</b> This may be different than {@link #getTargetVelocity()}.
     *         </p>
     */
    @Override
    public Measure<Voltage> getCurrentVoltage() {
        return MathUtils.average(leftMotor.getCurrentVoltage(), rightMotor.getCurrentVoltage());
    }

    /**
     * @return The voltage that is currently being set to the left motor.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetVelocityLeft()}.
     *         </p>
     */
    public Measure<Voltage> getCurrentVoltageLeft() {
        return leftMotor.getCurrentVoltage();
    }

    /**
     * @return The voltage that is currently being set to the right motor.
     *         <p>
     *         <b>Note:</b> This may be different than
     *         {@link #getTargetVelocityRight()}.
     *         </p>
     */
    public Measure<Voltage> getCurrentVoltageRight() {
        return rightMotor.getCurrentVoltage();
    }

    /**
     * @return The value that was last sent to {@link #setPosition(Rotation2d)};
     *         <p>
     *         or the average between the 2 motor target positions, if they differ.
     *         </p>
     */
    @Override
    public Rotation2d getTargetPosition() {
        return MathUtils.average(leftMotor.getTargetPosition(), rightMotor.getTargetPosition());
    }

    /**
     * @return The value that was last sent to {@link #setPosition(Rotation2d)} or
     *         {@link #setPositionLeft(Rotation2d)};
     */
    public Rotation2d getTargetPositionLeft() {
        return leftMotor.getTargetPosition();
    }

    /**
     * @return The value that was last sent to {@link #setPosition(Rotation2d)} or
     *         {@link #setPositionRight(Rotation2d)};
     */
    public Rotation2d getTargetPositionRight() {
        return rightMotor.getTargetPosition();
    }

    /**
     * @return The value that was last sent to {@link #setVelocity(Measure)};
     *         <p>
     *         or the average between the 2 motor target velocities, if they differ.
     *         </p>
     */
    @Override
    public Measure<Velocity<Angle>> getTargetVelocity() {
        return MathUtils.average(leftMotor.getTargetVelocity(), rightMotor.getTargetVelocity());
    }

    /**
     * @return The value that was last sent to {@link #setVelocity(Measure)} or
     *         {@link #setVelocityLeft(Rotation2d)};
     */
    public Measure<Velocity<Angle>> getTargetVelocityLeft() {
        return leftMotor.getTargetVelocity();
    }

    /**
     * @return The value that was last sent to {@link #setVelocity(Measure)} or
     *         {@link #setVelocityRight(Rotation2d)};
     */
    public Measure<Velocity<Angle>> getTargetVelocityRight() {
        return rightMotor.getTargetVelocity();
    }

    /**
     * @return The value that was last sent to {@link #setVoltage(Measure)};
     *         <p>
     *         or the average between the 2 motor target voltages, if they differ.
     *         </p>
     */
    @Override
    public Measure<Voltage> getTargetVoltage() {
        return MathUtils.average(leftMotor.getTargetVoltage(), rightMotor.getTargetVoltage());
    }

    /**
     * @return The value that was last sent to {@link #setVoltage(Measure)}; or
     *         {@link #setVoltageLeft(Rotation2d)};
     */
    public Measure<Voltage> getTargetVoltageLeft() {
        return leftMotor.getTargetVoltage();
    }

    /**
     * @return The value that was last sent to {@link #setVoltage(Measure)}; or
     *         {@link #setVoltageRight(Rotation2d)};
     */
    public Measure<Voltage> getTargetVoltageRight() {
        return rightMotor.getTargetVoltage();
    }

    /**
     * @return {@code true}, if either motor is above the Maximum operating
     *         temperature minus the
     *         {@link Constants#getMotorSafeTemperatureBuffer()}.
     */
    @Override
    public boolean isTemperatureTooHigh() {
        return leftMotor.isTemperatureTooHigh() || rightMotor.isTemperatureTooHigh();
    }

    // -------------------- Setters --------------------

    @Override
    public void setPosition(Rotation2d position) {
        leftMotor.setPosition(position);
        rightMotor.setPosition(position);
    }

    /**
     * Runs the left motor to a given position.
     * 
     * @param position The position to run the motor to.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor in the
     *                 "forwards" direction, and a negative value runs the motor in
     *                 the "reverse" direction.
     *                 </p>
     */
    public void setPositionLeft(Rotation2d position) {
        leftMotor.setPosition(position);
    }

    /**
     * Runs the right motor to a given position.
     * 
     * @param position The position to run the left motor to.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor in the
     *                 "forwards" direction, and a negative value runs the motor in
     *                 the "reverse" direction.
     *                 </p>
     */
    public void setPositionRight(Rotation2d position) {
        rightMotor.setPosition(position);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        leftMotor.setVelocity(velocity);
        rightMotor.setVelocity(velocity);
    }

    /**
     * Runs the left motor at the given velocity.
     * 
     * @param velocity The speed at which to run the motor at.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor in the
     *                 "forwards" direction, and a negative value runs the motor
     *                 in the "reverse" direction.
     *                 </p>
     */
    public void setVelocityLeft(Measure<Velocity<Angle>> velocity) {
        leftMotor.setVelocity(velocity);
    }

    /**
     * Runs the right motor at the given velocity.
     * 
     * @param velocity The speed at which to run the motor at.
     *                 <p>
     *                 <i>Note:</i> A positive value runs the motor in the
     *                 "forwards" direction, and a negative value runs the motor
     *                 in the "reverse" direction.
     *                 </p>
     */
    public void setVelocityRight(Measure<Velocity<Angle>> velocity) {
        rightMotor.setVelocity(velocity);
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    /**
     * Runs the left motor at the given voltage.
     * 
     * @param voltage How many volts to send to the motor.
     *                <p>
     *                <i>Note:</i> A positive value runs the motor in the
     *                "forwards" direction, and a negative value runs the motor
     *                in the "reverse" direction.
     *                </p>
     */
    public void setVoltageLeft(Measure<Voltage> voltage) {
        leftMotor.setVoltage(voltage);
    }

    /**
     * Runs the right motor at the given voltage.
     * 
     * @param voltage How many volts to send to the motor.
     *                <p>
     *                <i>Note:</i> A positive value runs the motor in the
     *                "forwards" direction, and a negative value runs the motor
     *                in the "reverse" direction.
     *                </p>
     */
    public void setVoltageRight(Measure<Voltage> voltage) {
        rightMotor.setVoltage(voltage);
    }

    // ========================= Commands ======================================

    /**
     * @return A {@link Command} that calls {@link #forwardLeftMotor} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    public Command forwardLeftMotorThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::forwardLeftMotor, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #forwardRightMotor} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    public Command forwardRightMotorThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::forwardRightMotor, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #reverseLeftMotor} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    public Command reverseLeftMotorThenStopCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::reverseLeftMotor, this::stop, this));
    }

    /**
     * @return A {@link Command} that calls {@link #reverseRightMotor} on
     *         {@link Command#initialize()} and {@link #stop()} on
     *         {@link Command#end()}.
     */
    public Command reverseRightMotorThenStopMotorCommand() {
        return CommandUtils.addName(getName(), new StartEndCommand(this::reverseRightMotor, this::stop, this));
    }
}
