package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;
    private Color[] dynamicPattern;
    private boolean runningDynamicPattern;
    private boolean isDynamicPatternFowards;

    /**
     * Initialize the {@link AddressableLED}, {@link AddresableLEDBuffer}, and
     * starts LEDs
     */
    public LightsSubsystem() {
        addressableLED = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
        addressableLED.setLength(Constants.ADDRESSABLE_LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(Constants.ADDRESSABLE_LED_LENGTH);
        addressableLED.start();

    }

    /**
     * Sets all lights to a static monocolor
     * 
     * @param color {@link Color} the lights are being set to
     */
    public void setStaticColor(Color color) {
        disableDynamicPattern();
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Disables dynamic patterns and sets all lights to a static multicolor pattern
     * 
     * @param pattern Array of {@link Color}s the lights are being set to
     */
    public void setStaticPattern(Color[] pattern) {
        disableDynamicPattern();
        setStaticPatternHelper(pattern);
    }

    /**
     * Sets all lights to a static multicolor pattern
     * 
     * @param pattern Array of {@link Color}s the lights are being set to
     */
    private void setStaticPatternHelper(Color[] pattern) {
        if (pattern.length == 0) {
            throw new RuntimeException("Pattern size may not be 0");
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, pattern[i % pattern.length]);
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Sets a pattern that scrolls either fowards or backwards
     * 
     * @param pattern                 Array of {@link Color}s to be set and scrolled
     *                                through
     * @param isDynamicPatternFowards Scroll fowards when {@code true}, backwards
     *                                when {@code false}
     */
    public void setDynamicPattern(Color[] pattern, boolean isDynamicPatternFowards) {
        dynamicPattern = pattern;
        runningDynamicPattern = true;
        this.isDynamicPatternFowards = isDynamicPatternFowards;
    }

    /**
     * Disables dynamic pattern, does not change lights
     */
    private void disableDynamicPattern() {
        runningDynamicPattern = false;
    }

    /**
     * Disables dynamic patterns, turns off lights
     */
    public void turnOff() {
        disableDynamicPattern();
        setStaticColor(new Color(0, 0, 0));
    }

    /**
     * Increase or decreases the brightness of the colors currently set to the
     * {@link AddressableLEDBuffer} by 10%
     * 
     * @param isDimming Decreases brightness when {@code true} and increases when
     *                  {@code false}
     */
    public void changeBrightness(boolean isDimming) {
        double factor = isDimming ? .9 : 1.1;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            Color currentColor = ledBuffer.getLED(i);
            ledBuffer.setLED(i, new Color(currentColor.red * factor, currentColor.green * factor,
                    currentColor.blue * factor));
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Takes dynamic pattern and scrolls colors by 1
     * 
     * @param pattern         An Array of {@link Color}s to be shifted
     * @param isScrollFowards Shifts colors fowads when {@code true} backwards when
     *                        {@code false}
     * @return Shifted array of colors
     */
    public static Color[] scrollPattern(Color[] pattern, boolean isScrollFowards) {
        Color[] temp = new Color[pattern.length];
        if (isScrollFowards) {
            for (int i = 0; i < pattern.length; i++) {

                if (i == pattern.length - 1) {
                    temp[0] = pattern[pattern.length - 1];
                } else {
                    temp[i + 1] = pattern[i];
                }
            }
        } else {
            for (int i = 0; i < pattern.length; i++) {
                if (i == 0) {
                    temp[temp.length - 1] = pattern[0];
                } else {
                    temp[i - 1] = pattern[i];
                }
            }
        }
        return temp;
    }

    @Override
    public void periodic() {
        if (runningDynamicPattern) {
            setStaticPatternHelper(dynamicPattern);
            dynamicPattern = scrollPattern(dynamicPattern, isDynamicPatternFowards);
        }
    }
}
