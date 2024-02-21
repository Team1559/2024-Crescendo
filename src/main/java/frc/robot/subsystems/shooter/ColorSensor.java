package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.AbstractConstants;

public class ColorSensor extends SubsystemBase {

    @AutoLog
    static class ColorSensorInputs {
        public double proximity;
        public boolean isObjectDetected;
        public int red, blue, green;
    }

    private ColorSensorInputsAutoLogged inputs = new ColorSensorInputsAutoLogged();
    private final ColorSensorV3 colorSensor;

    public ColorSensor(I2C.Port port) {
        colorSensor = new ColorSensorV3(port);
    }

    public ColorSensor() {
        this(I2C.Port.kOnboard);
    }

    @Override
    public void periodic() {
        updateInputs();
        isObjectDetected();
        Logger.processInputs("Shooter/Color Sensor", inputs);
    }

    private void updateInputs() {
        inputs.proximity = getProximity();
        inputs.isObjectDetected = isObjectDetected();
        inputs.red = colorSensor.getRed();
        inputs.blue = colorSensor.getBlue();
        inputs.green = colorSensor.getGreen();
    }

    // ========================= Functions =========================
    /**
     * Gets the color the {@link ColorSensorV3} is currently reading.
     * 
     * @return {@link Color} read.
     */
    public Color getColor() {
        return colorSensor.getColor();
    }

    /**
     * Gets the proximity (2047 closest, 0 farthest) the {@link ColorSensorV3} is
     * currently reading.
     * 
     * @return Proximity read.
     */
    public int getProximity() {
        return colorSensor.getProximity();
    }

    /**
     * Detect an object by comparing the current proximity to the proximity of the
     * {@link AbstractConstants#getColorSensorProximityThreshold} constant.
     * 
     * @return True if an object is detected
     */
    public boolean isObjectDetected() {
        inputs.isObjectDetected = colorSensor.getProximity() >= CONSTANTS.getColorSensorProximityThreshold();
        return inputs.isObjectDetected;
    }

    /**
     * Compares the currently observed color to the color passed in.
     * 
     * @param color Color for the sensed color to be compared to
     * 
     * @return The difference in color from a range of 0 - 765.
     * @author Kyle Holtz
     */
    public int compareToSensor(Color color) {
        int difference = 0;
        difference += Math.abs(color.red * 255 - colorSensor.getRed());
        difference += Math.abs(color.green * 255 - colorSensor.getGreen());
        difference += Math.abs(color.blue * 255 - colorSensor.getBlue());
        return difference;
    }

    // ========================= Commands =========================
    public Command waitForObjectCommand() {
        return new WaitUntilCommand(this::isObjectDetected);
    }

    public Command waitForNoObjectCommand() {
        return new WaitUntilCommand(() -> !isObjectDetected());
    }

    public Command getProximityCommand() {
        return new InstantCommand(this::getProximity);
    }
}
