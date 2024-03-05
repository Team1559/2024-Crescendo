package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.CONSTANTS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.vision.VisionInputsAutoLogged;
import frc.robot.io.vision.VisionIo;

public class Vision extends SubsystemBase {

    private final VisionIo[] ios;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

    public Vision(SwerveDrivePoseEstimator poseEstimator, VisionIo... ios) {
        this.poseEstimator = poseEstimator;
        this.ios = ios;
    }

    @Override
    public void periodic() {

        for (VisionIo io : ios) {

            io.updateInputs(inputs);
            Logger.processInputs("Vision/" + io.getName(), inputs);

            if (inputs.havePose) {

                Vector<N3> estimateStandardDeviations = VecBuilder.fill(
                        inputs.distanceToTarget.in(Meters) * CONSTANTS.getCameraLinearStandardDeviation().in(Meters),
                        inputs.distanceToTarget.in(Meters) * CONSTANTS.getCameraLinearStandardDeviation().in(Meters),
                        CONSTANTS.getCameraRotationalStandardDeviation().in(Degrees));

                poseEstimator.addVisionMeasurement(inputs.pose, inputs.timestamp, estimateStandardDeviations);
            }
        }
    }
}
