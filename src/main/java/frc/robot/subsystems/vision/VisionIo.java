package frc.robot.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.Vision.VisionInputs;

public interface VisionIo {
    public void updateInputs(VisionInputs inputs);

    public Vector<N3> getEstimateStdDevs();
}
