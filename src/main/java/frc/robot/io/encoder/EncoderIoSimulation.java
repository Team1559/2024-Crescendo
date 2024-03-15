package frc.robot.io.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.io.motor.MotorIoSimulation;

/**
 * Uses a {@link MotorIoSimulation} object to simulate an encoder.
 */
public class EncoderIoSimulation implements EncoderIo {

    public final MotorIoSimulation motorIoSimulation;

    public EncoderIoSimulation(MotorIoSimulation motorIoSimulation) {

        // ---------- Create & Configure Encoder ----------
        this.motorIoSimulation = motorIoSimulation;
    }

    @Override
    public void updateInputs(EncoderIoInputs inputs) {
        inputs.positionAbsolute = getAbsolutePosition();
        inputs.positionRelative = getRelativePosition();
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getAbsolutePosition() {
        return motorIoSimulation.getPositionAbsolute();
    }

    @Override
    public Rotation2d getRelativePosition() {
        return motorIoSimulation.getPositionRelative();
    }

    @Override
    public boolean isInverted() {
        return false;
    }

    /** Does Nothing. */
    @Override
    public void setOffset(Rotation2d offset) {
        // No functionality.
    }
}
