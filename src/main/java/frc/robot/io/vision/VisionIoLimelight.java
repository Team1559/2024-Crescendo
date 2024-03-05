package frc.robot.io.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;

public class VisionIoLimelight implements VisionIo {

    private String cameraName;

    public VisionIoLimelight(String cameraName) {
        this.cameraName = cameraName;
    }

    @Override
    public String getName() {
        return this.cameraName;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {

        double[] data = LimelightHelpers.getBotPose_wpiBlue(cameraName);
        if (data.length < 6 || (data[0] == 0 && data[1] == 0)) {

            inputs.havePose = false;
            inputs.pose = new Pose2d();
            inputs.timestamp = 0;
        } else {

            double[] targetData = LimelightHelpers.getTargetPose_CameraSpace(cameraName);

            inputs.distanceToTarget = Meters.of(Math.hypot(targetData[0], targetData[1]));

            inputs.havePose = true;

            // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
            inputs.pose = new Pose2d(new Translation2d(data[0], data[1]), Rotation2d.fromDegrees(data[5]));

            inputs.timestamp = Timer.getFPGATimestamp() - data[6] / 1000.0;
        }
    }
}
