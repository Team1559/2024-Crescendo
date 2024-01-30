package frc.robot.subsystems.colorsensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase{
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  public ColorSensor(){

  }
  public Color getColor(){
    return colorSensor.getColor();
  }
  public int compareToSensor(Color color){
    int difference = 0;
    difference += Math.abs(color.red - colorSensor.getRed());
    difference += Math.abs(color.green - colorSensor.getGreen());
    difference += Math.abs(color.blue - colorSensor.getBlue());
    return difference;
  }
}
