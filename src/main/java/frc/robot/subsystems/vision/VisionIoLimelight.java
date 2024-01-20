package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.Vision.VisionInputs;

public class VisionIoLimelight implements VisionIo {
    private final static double X_STDEV = 1.0;
    private final static double Y_STDEV = 1.0;
    private final static double R_STDEV = 2.0;
    private String cameraName;

    public VisionIoLimelight(String cameraName) {
        this.cameraName = cameraName;
    }

    public void updateInputs(VisionInputs inputs) {
        var data = LimelightHelpers.getBotPose(cameraName);

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
            inputs.timestamp = Timer.getFPGATimestamp() - data[6] / 1000.0;
        }
    }

    public Vector<N3> getEstimateStdDevs() {
        return VecBuilder.fill(X_STDEV, Y_STDEV, R_STDEV);
    }
}
