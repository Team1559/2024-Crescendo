package org.victorrobotics.frc.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.vision.VisionInputsAutoLogged;
import org.victorrobotics.frc.io.vision.VisionIo;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

                double weightedLinearStandardDeviation = inputs.distanceToTarget.in(Meters)
                        * Constants.getCameraLinearStandardDeviation().in(Meters) / inputs.numberOfTargets;

                Vector<N3> estimateStandardDeviations = VecBuilder.fill(weightedLinearStandardDeviation,
                        weightedLinearStandardDeviation, Constants.getCameraRotationalStandardDeviation().in(Degrees));

                poseEstimator.addVisionMeasurement(inputs.pose, inputs.timestamp, estimateStandardDeviations);
            }
        }
    }
}
