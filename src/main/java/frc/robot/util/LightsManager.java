package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LightsManager extends AddressableLED {

    private AddressableLEDBuffer buffer;

    public LightsManager(int port, int length) {
        super(port);
        buffer = new AddressableLEDBuffer(length);
        this.setLength(buffer.getLength());
        this.start();
    }

    public void setColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        this.setData(buffer);
    }

    public void setRGB(int red, int green, int blue) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, red, green, blue);
        }
        this.setData(buffer);
    }

    public void setHSV(int hue, int saturation, int value){
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, hue, saturation, value);
        }
        this.setData(buffer);
    }

    public void setOff(){
        for (int i = 0; i < buffer.getLength(); i++){
            buffer.setHSV(i, 0, 0, 0);
        }
        this.setData(buffer);
    }

    public int getLength() {
        return buffer.getLength();
    }

}
