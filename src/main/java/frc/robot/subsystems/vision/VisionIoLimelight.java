package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.Vision.VisionInputs;

public class VisionIoLimelight implements VisionIo {
    private final double[] STD_DEVS = { 1.0, 1.0, 2.0 };
    private String cameraName;

    public VisionIoLimelight(String cameraName) {
        this.cameraName = cameraName;
    }

    public String name() {
        return this.cameraName;
    }

    public void updateInputs(VisionInputs inputs) {
        double[] data = LimelightHelpers.getBotPose(cameraName);

        if (data.length < 6) {
            inputs.havePose = false;
            inputs.pose = new Pose2d();
            inputs.timestamp = 0;
        } else {
            // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
            Translation2d t = new Translation2d(data[0], data[1]);
            Rotation2d r = Rotation2d.fromDegrees(data[5]);

            inputs.havePose = true;
            inputs.pose = new Pose2d(t, r);
            inputs.estimateStdDevs = STD_DEVS;
            inputs.timestamp = Timer.getFPGATimestamp() - data[6] / 1000.0;
        }
    }
}
