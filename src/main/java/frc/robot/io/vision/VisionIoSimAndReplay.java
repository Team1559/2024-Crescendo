package frc.robot.io.vision;

public class VisionIoSimAndReplay implements VisionIo {

    @Override
    public String getName() {
        return "sim_replay";
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        // Don't need to do anything here.
    }
}
