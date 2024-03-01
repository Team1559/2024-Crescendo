package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.Vision.VisionInputs;

public class VisionIoLimelight implements VisionIo {

    private static final double LINEAR_STD_DEV_RATIO = 0.5;
    private static final double ROTATION_STD_DEV = 1;

    private String cameraName;

    public VisionIoLimelight(String cameraName) {
        this.cameraName = cameraName;
    }

    public String name() {
        return this.cameraName;
    }

    public void updateInputs(VisionInputs inputs) {
        double[] data = LimelightHelpers.getBotPose_wpiBlue(cameraName);
        if (data.length < 6 || (data[0] == 0 && data[1] == 0)) {
            inputs.havePose = false;
            inputs.pose = new Pose2d();
            inputs.timestamp = 0;
        } else {
            // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
            Translation2d t = new Translation2d(data[0], data[1]);
            Rotation2d r = Rotation2d.fromDegrees(data[5]);

            double[] targetdata = LimelightHelpers.getTargetPose_CameraSpace(cameraName);
            inputs.distanceToTarget = Math.hypot(targetdata[0], targetdata[1]);
            inputs.estimateStdDevs[0] = inputs.distanceToTarget * LINEAR_STD_DEV_RATIO;
            inputs.estimateStdDevs[1] = inputs.distanceToTarget * LINEAR_STD_DEV_RATIO;
            inputs.estimateStdDevs[2] = ROTATION_STD_DEV;

            inputs.havePose = true;
            inputs.pose = new Pose2d(t, r);
            inputs.timestamp = Timer.getFPGATimestamp() - data[6] / 1000.0;
        }
    }
}
