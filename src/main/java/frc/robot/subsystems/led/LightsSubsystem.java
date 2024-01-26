package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;

    public LightsSubsystem() {
        addressableLED = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
        addressableLED.setLength(Constants.ADDRESSABLE_LED_LENGTH);
        addressableLED.start();
    }

    public void setStaticColor(Color color) {
        ledBuffer = new AddressableLEDBuffer(Constants.ADDRESSABLE_LED_LENGTH);
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        addressableLED.setData(ledBuffer);
    }

    public void turnOff() {
        setStaticColor(new Color(0, 0, 0));
    }

    public void changeBrightness(boolean isDimming) {
        double factor = isDimming ? .9 : 1.1;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            Color currentColor = ledBuffer.getLED(i);
            ledBuffer.setLED(i, new Color(currentColor.red * factor, currentColor.green * factor,
                    currentColor.blue * factor));
        }
        addressableLED.setData(ledBuffer);
    }
}
