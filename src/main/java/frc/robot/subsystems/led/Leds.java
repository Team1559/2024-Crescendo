package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CommandUtils;

public class Leds extends SubsystemBase {

    // ========================= Class Level =========================

    /**
     * Takes dynamic pattern and scrolls colors by 1.
     * 
     * @param pattern          An Array of {@link Color}s to be shifted.
     * @param isScrollForwards Shifts colors forwards w.hen {@code true} backwards
     *                         when
     *                         {@code false}
     * @param updateDelay      decreases the number of updates based on update
     *                         delay,
     *                         updates every nth attempt.
     * @return Shifted array of colors.
     */
    private static Color[] scrollPattern(Color[] pattern, boolean isScrollForwards) {
        Color[] tempArray = pattern;
        tempArray = new Color[pattern.length];
        if (isScrollForwards) {
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

    private int dynamicColorCounter = 0;
    private boolean isDynamicPatternForwards;

    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;
    private Runnable endDynamicPatternCallback;

    private Color[] dynamicPattern, pausedDynamicPattern;

    /**
     * Initialize the {@link AddressableLED}, {@link AddressableLEDBuffer}, and
     * starts LEDs.
     */
    public Leds() {
        addressableLED = new AddressableLED(Constants.getLedPort());
        addressableLED.setLength(Constants.getLedLength());
        ledBuffer = new AddressableLEDBuffer(Constants.getLedLength());
        addressableLED.start();
    }

    @Override
    public void periodic() {
        if (dynamicPattern != null) {
            setStaticPatternCallback(dynamicPattern);
            if (++dynamicColorCounter % 3 /* larger number = slower scrolling speed */ == 0) {
                dynamicPattern = scrollPattern(dynamicPattern, isDynamicPatternForwards);
            }
        } else if (endDynamicPatternCallback != null) {
            endDynamicPatternCallback.run();
            endDynamicPatternCallback = null;
        }
    }

    // ========================= Functions =====================================

    /**
     * Increase or decreases the brightness of the colors currently set to the
     * {@link AddressableLEDBuffer} by 15%.
     * 
     * @param isDimming Decreases brightness when {@code true} and increases when
     *                  {@code false}.
     */
    public void changeBrightness(boolean isDimming) {
        if (dynamicPattern != null) {
            pausedDynamicPattern = dynamicPattern;
        }
        stopDynamicPattern(() -> changeBrightnessCallback(isDimming));
    }

    public void changeBrightnessCallback(boolean isDimming) {

        // ---------- Change Brightness ----------
        double factor = isDimming ? .85 : 1.15;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            Color currentColor = ledBuffer.getLED(i);
            ledBuffer.setLED(i,
                    new Color(currentColor.red * factor, currentColor.green * factor, currentColor.blue * factor));
        }
        addressableLED.setData(ledBuffer);

        // ---------- Reenable Dynamic Pattern ----------
        if (pausedDynamicPattern != null) {
            setDynamicPattern(pausedDynamicPattern, isDynamicPatternForwards);
            pausedDynamicPattern = null;
        }
    }

    /**
     * Sets a pattern that scrolls either forwards or backwards.
     * <i>Notes:</i>
     * </p>
     * <ul>
     * <li>If the patters does not fit into the LEDs, it will be truncated.</li>
     * <li>{@link Color#kBlack} can be used to separate the pattern.</li>
     * </ul>
     * 
     * @param pattern                  {@link Color}s to be scrolled through.
     * @param isDynamicPatternForwards Scroll forwards when {@code true} or
     *                                 backwards when {@code false}.
     */
    public void setDynamicPattern(Color[] pattern, boolean isDynamicPatternForwards) {
        this.isDynamicPatternForwards = isDynamicPatternForwards;
        dynamicPattern = pattern;
    }

    /**
     * Sets all lights to a single color.
     * 
     * @param color {@link Color} the lights are being set to.
     */
    public void setColor(Color color) {
        stopDynamicPattern(() -> setColorCallback(color));
    }

    private void setColorCallback(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        addressableLED.setData(ledBuffer);
    }

    public void setAllianceColor() {
        stopDynamicPattern(() -> setColorCallback(Constants.getAllianceColor()));
    }

    /**
     * Sets all lights to a static multicolor pattern. This pattern will be repeated
     * across the LEDs.
     * <p>
     * <i>Notes:</i>
     * </p>
     * <ul>
     * <li>If the patters does not fit evenly into the LEDs, it will be
     * truncated.</li>
     * <li>{@link Color#kblack} can be used to sparate the poattern.</li>
     * </ul>
     * 
     * @param pattern Array of {@link Color}s the lights are being set to.
     */
    public void setStaticPattern(Color[] pattern) {
        stopDynamicPattern(() -> setStaticPatternCallback(pattern));
    }

    private void setStaticPatternCallback(Color[] pattern) {
        if (pattern.length == 0) {
            throw new RuntimeException("Pattern size may not be 0");
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, pattern[i % pattern.length]);
        }
        addressableLED.setData(ledBuffer);
    }

    /**
     * Disables dynamic pattern, does not change lights.
     */
    private void stopDynamicPattern(Runnable endDynamicPatternCallback) {
        this.endDynamicPatternCallback = endDynamicPatternCallback;
        dynamicPattern = null;
    }

    public void turnOff() {
        stopDynamicPattern(() -> setColorCallback(Color.kBlack));
    }

    // ========================= Function Commands =============================

    /**
     * Dims/Brightens the lights
     * 
     * @param isDimming are lights being dimmed or brightened
     * @return
     */
    public Command changeBrightnessCommand(boolean isDimming) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> changeBrightness(isDimming), this));
    }

    /**
     * Set the lights to a scrolling pattern
     * 
     * @param pattern                  Pattern the LEDs are being set to
     * @param isDynamicPatternForwards is the pattern scrolling forwards or
     *                                 backwards
     * @return
     */
    public Command setDynamicPatternCommand(Color[] pattern, boolean isDynamicPatternForwards) {
        return CommandUtils.addName(getName(),
                new InstantCommand(() -> setDynamicPattern(pattern, isDynamicPatternForwards), this));
    }

    /**
     * Set color of the LEDs
     * 
     * @param color Color LEDs are being set to
     * @return
     */
    public Command setColorCommand(Color color) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setColor(color), this));
    }

    public Command setAllianceColorCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::setAllianceColor, this));
    }

    /**
     * Sets a static pattern to the LEDs
     * 
     * @param subsystem LEDs being set
     * @param pattern   Pattern being set to the LEDs
     * @return
     */
    public Command setStaticPatternCommand(Color[] pattern) {
        return CommandUtils.addName(getName(), new InstantCommand(() -> setStaticPattern(pattern), this));
    }

    public Command turnOffCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::turnOff, this));
    }
}
