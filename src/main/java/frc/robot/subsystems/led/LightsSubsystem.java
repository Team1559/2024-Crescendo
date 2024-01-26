package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
    
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;

    public LightsSubsystem() {
        addressableLED = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Constants.ADDRESSABLE_LED_LENGTH);
        addressableLED.setLength(ledBuffer.getLength());
        addressableLED.start();
    }

    


    public void setStaticColor(Color color){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setLED(i, color);
        }
        addressableLED.setData(ledBuffer);
    }
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
