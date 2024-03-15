package frc.robot.io.encoder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncoderIo {

    @AutoLog
    static class EncoderIoInputs {

        public String[] faults = new String[0];

        public Rotation2d positionAbsolute = null;
        public Rotation2d positionRelative = null;
    }

    public void updateInputs(EncoderIoInputs inputs);

    // ========================= Functions =========================

    public Rotation2d getAbsolutePosition();

    public Rotation2d getRelativePosition();

    public boolean isInverted();

    public void setOffset(Rotation2d offset);
}
