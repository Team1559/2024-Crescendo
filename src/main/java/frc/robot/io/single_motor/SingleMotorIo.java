package frc.robot.io.single_motor;

import org.littletonrobotics.junction.AutoLog;

public interface SingleMotorIo {
    
    @AutoLog
    static class SingleMotorIoInputs {
        public double appliedOutput;
        public double outputCurrent;
        public double motorTemp;
        public int faults;
        public double velocity;
    }

    public void updateInputs(SingleMotorIoInputs inputs);

    public void setVoltage(double voltage);
}
