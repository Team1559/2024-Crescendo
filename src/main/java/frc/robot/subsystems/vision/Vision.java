package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

public class Vision {
    private final VisionIo io;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

    @AutoLog
    public static class VisionInputs {
        public boolean havePose = false;
        public Pose2d pose = new Pose2d();
        public double timestamp;
    }

    public Vision(VisionIo io, SwerveDrivePoseEstimator poseEstimator) {
        this.io = io;
        this.poseEstimator = poseEstimator;

        poseEstimator.setVisionMeasurementStdDevs(io.getEstimateStdDevs());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        if (inputs.havePose) {
            poseEstimator.addVisionMeasurement(inputs.pose, inputs.timestamp);
        }
    }
}
