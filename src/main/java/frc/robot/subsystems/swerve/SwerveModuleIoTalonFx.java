package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder.
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax").
 * </p>
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad".
 * </p>
 */
public class SwerveModuleIoTalonFx implements SwerveModuleIo {

  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder cancoder;

  private final StatusSignal<Double> driveMotorPosition;
  private final StatusSignal<Double> driveMotorVelocity;
  private final StatusSignal<Double> driveMotorAppliedVolts;
  private final StatusSignal<Double> driveMotorCurrent;
  private final StatusSignal<Integer> driveMotorFaults;
  private final StatusSignal<Double> driveMotorTemp;

  private final StatusSignal<Double> cancoderAbsolutePosition;
  private final StatusSignal<Double> steerMotorPosition;
  private final StatusSignal<Double> steerMotorVelocity;
  private final StatusSignal<Double> steerMotorAppliedVolts;
  private final StatusSignal<Double> steerMotorStatorCurrent;
  private final StatusSignal<Integer> steerMotorFaults;
  private final StatusSignal<Double> steerMotorTemp;

  private final Rotation2d absoluteEncoderOffset;

  public SwerveModuleIoTalonFx(WheelModuleIndex index) {

    // Assign Motor and Encoder Ids and configue wheel offset.
    switch (index) {
      case FRONT_LEFT:
        driveMotor = new TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        steerMotor = new TalonFX(Constants.FRONT_LEFT_STEER_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        cancoder = new CANcoder(Constants.FRONT_LEFT_CANCODER_ID, Constants.CANIVORE_BUS_ID);
        absoluteEncoderOffset = Constants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET;
        break;
      case FRONT_RIGHT:
        driveMotor = new TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        steerMotor = new TalonFX(Constants.FRONT_RIGHT_STEER_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        cancoder = new CANcoder(Constants.FRONT_RIGHT_CANCODER_ID, Constants.CANIVORE_BUS_ID);
        absoluteEncoderOffset = Constants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET;
        break;
      case BACK_LEFT:
        driveMotor = new TalonFX(Constants.BACK_LEFT_DRIVE_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        steerMotor = new TalonFX(Constants.BACK_LEFT_STEER_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        cancoder = new CANcoder(Constants.BACK_LEFT_CANCODER_ID, Constants.CANIVORE_BUS_ID);
        absoluteEncoderOffset = Constants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET;
        break;
      case BACK_RIGHT:
        driveMotor = new TalonFX(Constants.BACK_RIGHT_DRIVE_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        steerMotor = new TalonFX(Constants.BACK_RIGHT_STEER_MOTOR_ID, Constants.CANIVORE_BUS_ID);
        cancoder = new CANcoder(Constants.BACK_RIGHT_CANCODER_ID, Constants.CANIVORE_BUS_ID);
        absoluteEncoderOffset = Constants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET;
        break;
      default:
        throw new RuntimeException("Invalid module index: " + index);
    }

    // Set Drive TalonFXConfiguration.
    var driveTalonFXConfiguration = new TalonFXConfiguration();
    driveTalonFXConfiguration.CurrentLimits = Constants.getDefaultCurrentLimitsConfig();
    driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

    // Set Drive TalonFXConfiguration.
    var driveMotorOutputConfigs = new MotorOutputConfigs();
    driveMotorOutputConfigs.NeutralMode = Constants.WHEEL_BRAKE_MODE;
    // Inverted to match our Swerve Drive Module Gear Box & Motors.
    driveMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    driveMotor.getConfigurator().apply(driveMotorOutputConfigs);

    // Set Steer MotorOutputConfigs.
    var steerTalonFXConfiguration = new TalonFXConfiguration();
    steerTalonFXConfiguration.CurrentLimits = Constants.getDefaultCurrentLimitsConfig();
    steerMotor.getConfigurator().apply(steerTalonFXConfiguration);

    // Set Steer MotorOutputConfigs.
    var steerMotorOutputConfigs = new MotorOutputConfigs();
    steerMotorOutputConfigs.NeutralMode = Constants.WHEEL_BRAKE_MODE;
    // Inverted to match our Swerve Drive Module Gear Box & Motors.
    steerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    steerMotor.getConfigurator().apply(steerMotorOutputConfigs);

    // Set CAN Coder Configs.
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    cancoder.getConfigurator().apply(canCoderConfiguration);

    // Get current state.
    driveMotorPosition = driveMotor.getPosition();
    driveMotorVelocity = driveMotor.getVelocity();
    driveMotorAppliedVolts = driveMotor.getMotorVoltage();
    driveMotorCurrent = driveMotor.getStatorCurrent();
    driveMotorFaults = driveMotor.getFaultField();
    driveMotorTemp = driveMotor.getDeviceTemp();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    steerMotorPosition = steerMotor.getPosition();
    steerMotorVelocity = steerMotor.getVelocity();
    steerMotorAppliedVolts = steerMotor.getMotorVoltage();
    steerMotorStatorCurrent = steerMotor.getStatorCurrent();
    steerMotorFaults = steerMotor.getFaultField();
    steerMotorTemp = steerMotor.getDeviceTemp();

    driveMotor.optimizeBusUtilization();
    steerMotor.optimizeBusUtilization();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    steerMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void updateInputs(SwerveModuleIoInputs inputs) {
    BaseStatusSignal.refreshAll(
        driveMotorPosition,
        driveMotorVelocity,
        driveMotorAppliedVolts,
        driveMotorCurrent,
        cancoderAbsolutePosition,
        steerMotorPosition,
        steerMotorVelocity,
        steerMotorAppliedVolts,
        steerMotorStatorCurrent,
        driveMotorFaults,
        steerMotorFaults,
        driveMotorTemp,
        steerMotorTemp);

    // Inverted driveMotorPosition so that sutonomous sees the robot moving in the
    // correct direction.
    inputs.driveMotorPositionRad = Units.rotationsToRadians(-driveMotorPosition.getValueAsDouble())
        / Constants.WHEEL_DRIVE_GEAR_RATIO;
    inputs.driveMotorVelocityRadPerSec = Units.rotationsToRadians(driveMotorVelocity.getValueAsDouble())
        / Constants.WHEEL_DRIVE_GEAR_RATIO;
    inputs.driveMotorAppliedVolts = driveMotorAppliedVolts.getValueAsDouble();
    inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();
    inputs.driveMotorFaults = driveMotorFaults.getValue();
    inputs.driveMotorTemp = driveMotorTemp.getValueAsDouble();

    inputs.cancoderAbsolutePosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble())
        .minus(absoluteEncoderOffset);
    inputs.steerMotorPosition = Rotation2d
        .fromRotations(steerMotorPosition.getValueAsDouble() / Constants.WHEEL_TURN_GEAR_RATIO);
    inputs.steerMotorVelocityRadPerSec = Units.rotationsToRadians(steerMotorVelocity.getValueAsDouble())
        / Constants.WHEEL_TURN_GEAR_RATIO;
    inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
    inputs.steerMotorCurrentAmps = steerMotorStatorCurrent.getValueAsDouble();
    inputs.steerMotorFaults = steerMotorFaults.getValue();
    inputs.steerMotorTemp = steerMotorTemp.getValueAsDouble();
  }
}