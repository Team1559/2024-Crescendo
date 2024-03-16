package org.victorrobotics.frc.io.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface GyroIo {

    @AutoLog
    public static class GyroIoInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Measure<Velocity<Angle>> yawVelocity = DegreesPerSecond.of(0);
    }

    public void updateInputs(GyroIoInputs inputs);
}