package frc.robot.subsystems.color_sensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

// TODO: Determine if all methods in this class should be static.
public class ColorSensor extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

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
   * {@link Constants#COLOR_SENSOR_PROXIMITY_THRESHOLD} constant.
   * 
   * @return True if an object is detected
   */
  public boolean isObjectDetected() {
    return colorSensor.getProximity() > Constants.COLOR_SENSOR_PROXIMITY_THRESHOLD;
  }

  /**
   * Compares the currently observed color to the color passed in.
   * 
   * @param color Color for the sensed color to be compared to
   * 
   * @return The difference in color from a range of 0 - 765.
   */
  public int compareToSensor(Color color) {
    int difference = 0;
    difference += Math.abs(color.red - colorSensor.getRed());
    difference += Math.abs(color.green - colorSensor.getGreen());
    difference += Math.abs(color.blue - colorSensor.getBlue());
    return difference;
  }

  // ========================= Commands =========================
  // TODO.
}