package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIo {
  
  @AutoLog
  public static class SwerveModuleIoInputs {
    public double driveMotorPositionRad;
    public double driveMotorVelocityRadPerSec;
    public double driveMotorAppliedVolts;
    public double driveMotorCurrentAmps;

    public Rotation2d cancoderAbsolutePosition = new Rotation2d();
    public Rotation2d steerMotorPosition = new Rotation2d();
    public double steerMotorVelocityRadPerSec;
    public double steerMotorAppliedVolts;
    public double steerMotorCurrentAmps;
  }

  /** Run the drive motor at the specified voltage. */
  public void setDriveVoltage(double volts);

  /** Run the turn motor at the specified voltage. */
  public void setTurnVoltage(double volts);

  /** Updates the set of loggable inputs. */
  public void updateInputs(SwerveModuleIoInputs inputs);
}