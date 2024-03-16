package org.victorrobotics.frc.io.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public interface VisionIo {

    @AutoLog
    public static class VisionInputs {
        public Measure<Distance> distanceToTarget;
        public boolean havePose = false;
        public int numberOfTargets = 0;
        public Pose2d pose = new Pose2d();
        public double timestamp;
        // public int[] fIds = new int[0];
    }

    public String getName();

    public void updateInputs(VisionInputs inputs);
}
