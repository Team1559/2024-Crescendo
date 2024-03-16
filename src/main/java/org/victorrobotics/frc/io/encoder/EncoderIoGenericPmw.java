package org.victorrobotics.frc.io.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * A generic implementation of the {@link EncoderIo} using the
 * {@link DutyCycleEncoder} class, for encoders that connect to a PMW port.
 */
public class EncoderIoGenericPmw implements EncoderIo {

    public final boolean isInverted;
    public final DutyCycleEncoder encoder;
    public final Rotation2d relativeOffset;

    public EncoderIoGenericPmw(int pmwPort, boolean isInverted, Rotation2d offset) {

        // ---------- Create & Configure Encoder ----------
        encoder = new DutyCycleEncoder(pmwPort);
        this.isInverted = isInverted;
        setOffset(offset);
        relativeOffset = getAbsolutePosition();
    }

    @Override
    public void updateInputs(EncoderIoInputs inputs) {
        inputs.positionAbsolute = getAbsolutePosition();
        inputs.positionRelative = getRelativePosition();
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getAbsolutePosition() {

        Rotation2d position = Rotation2d.fromRotations(encoder.getAbsolutePosition());

        if (isInverted) {
            position = position.unaryMinus();
        }

        return position;
    }

    @Override
    public Rotation2d getRelativePosition() {

        Rotation2d position = getAbsolutePosition().minus(relativeOffset);

        // Forces Rotation2d to determine if it has passed threshold (360 degrees).
        position.plus(Rotation2d.fromDegrees(0));

        return position;
    }

    @Override
    public boolean isInverted() {
        return isInverted;
    }

    @Override
    public void setOffset(Rotation2d offset) {
        encoder.setPositionOffset(offset.getRotations() % 1);
    }
}
