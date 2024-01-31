package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {

  // ========================= Class Level =========================
    /**
     * Takes dynamic pattern and scrolls colors by 1.
     * 
     * @param pattern         An Array of {@link Color}s to be shifted.
     * @param isScrollFowards Shifts colors fowads w.hen {@code true} backwards when
     *                        {@code false}
     * @return Shifted array of colors.
     */
    public static Color[] scrollPattern(Color[] pattern, boolean isScrollFowards) {
        Color[] tempArray = new Color[pattern.length];
        if (isScrollFowards) {
            for (int i = 0; i < pattern.length; i++) {
                if (i == pattern.length - 1) {
                    tempArray[0] = pattern[pattern.length - 1];
                } else {
                    tempArray[i + 1] = pattern[i];
                }
            }
        } else {
            for (int i = 0; i < pattern.length; i++) {
                if (i == 0) {
                    tempArray[tempArray.length - 1] = pattern[0];
                } else {
                    tempArray[i - 1] = pattern[i];
                }
            }
        }
        return tempArray;
    }

    // ========================= Object Level =========================
    private boolean isDynamicPatternFowards;
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;
    private Color[] dynamicPattern;

    /**
     * Initialize the {@link AddressableLED}, {@link AddresableLEDBuffer}, and
     * starts LEDs.
     */
    public LightsSubsystem() {
        addressableLED = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
        addressableLED.setLength(Constants.ADDRESSABLE_LED_LENGTH);
        ledBuffer = new AddressableLEDBuffer(Constants.ADDRESSABLE_LED_LENGTH);
        addressableLED.start();
    }

    /**
     * Increase or decreases the brightness of the colors currently set to the
     * {@link AddressableLEDBuffer} by 15%.
     * 
     * @param isDimming Decreases brightness when {@code true} and increases when
     *                  {@code false}.
     */
    public void changeBrightness(boolean isDimming) {
        double factor = isDimming ? .85 : 1.15;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            Color currentColor = ledBuffer.getLED(i);
            ledBuffer.setLED(i, new Color(currentColor.red * factor, currentColor.green * factor,
                    currentColor.blue * factor));
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Disables dynamic pattern, does not change lights.
     */
    private void disableDynamicPattern() {
        dynamicPattern = null;
    }

    @Override
    public void periodic() {
        if (dynamicPattern != null) {
            setStaticPatternHelper(dynamicPattern);
            dynamicPattern = scrollPattern(dynamicPattern, isDynamicPatternFowards);
        }
    }

    /**
     * Sets a pattern that scrolls either fowards or backwards.
     * <i>Notes:</i>
     * </p>
     * <ul>
     * <li>If the patters does not fit evenly into the LEDs, it will be truncated.</li>
     * <li>{@link Color#kblack} can be used to sparate the poattern.</li>
     * </ul>
     * 
     * @param pattern                 Array of {@link Color}s to be set and scrolled
     *                                through.
     * @param isDynamicPatternFowards Scroll fowards when {@code true}, backwards
     *                                when {@code false}.
     */
    public void setDynamicPattern(Color[] pattern, boolean isDynamicPatternFowards) {
        dynamicPattern = pattern;
        this.isDynamicPatternFowards = isDynamicPatternFowards;
    }

    /**
     * Sets all lights to a static monocolor.
     * 
     * @param color {@link Color} the lights are being set to.
     */
    public void setStaticColor(Color color) {
        disableDynamicPattern();
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Sets all lights to a static multicolor pattern. This pattern will be repeated arross the LEDs.
     * <p>
     * <i>Notes:</i>
     * </p>
     * <ul>
     * <li>If the patters does not fit evenly into the LEDs, it will be truncated.</li>
     * <li>{@link Color#kblack} can be used to sparate the poattern.</li>
     * </ul>
     * 
     * @param pattern Array of {@link Color}s the lights are being set to.
     */
    public void setStaticPattern(Color[] pattern) {
        disableDynamicPattern();
        setStaticPatternHelper(pattern);
    }

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
     * Disables dynamic patterns, turns off lights.
     */
    public void turnOff() {
        disableDynamicPattern();
        setStaticColor(Color.kBlack);
    }
}
