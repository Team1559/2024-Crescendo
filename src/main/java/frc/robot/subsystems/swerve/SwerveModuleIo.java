package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIo {

  @AutoLog
  static class SwerveModuleIoInputs {
    
    public Rotation2d cancoderOffsetPosition = new Rotation2d();
    public Rotation2d cancoderAbsolutePosition = new Rotation2d();

    public double driveMotorPositionRad;
    public double driveMotorVelocityRadPerSec;
    public double driveMotorAppliedVolts;
    public double driveMotorCurrentAmps;
    public int driveMotorFaults;
    public double driveMotorTemp;

    public Rotation2d steerMotorPosition = new Rotation2d();
    public double steerMotorVelocityRadPerSec;
    public double steerMotorAppliedVolts;
    public double steerMotorCurrentAmps;
    public int steerMotorFaults;
    public double steerMotorTemp;
  }

  /** Run the drive motor at the specified voltage. */
  public void setDriveVoltage(double volts);

  /** Run the turn motor at the specified voltage. */
  public void setTurnVoltage(double volts);

  /** Updates the set of loggable inputs. */
  public void updateInputs(SwerveModuleIoInputs inputs);
}