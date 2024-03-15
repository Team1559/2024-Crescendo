package frc.robot.io.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class EncoderIoReplay implements EncoderIo {

    public EncoderIoReplay() {
        // No functionality.
    }

    @Override
    public void updateInputs(EncoderIoInputs inputs) {
        // No functionality.
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromRotations(0);
    }

    @Override
    public Rotation2d getRelativePosition() {
        return Rotation2d.fromDegrees(0);
    }

    @Override
    public boolean isInverted() {
        return false;
    }

    @Override
    public void setOffset(Rotation2d offset) {
        // No functionality.
    }
}
