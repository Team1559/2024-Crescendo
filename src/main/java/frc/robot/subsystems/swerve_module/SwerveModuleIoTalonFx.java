package frc.robot.subsystems.swerve_module;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    private final StatusSignal<Double> cancoderAbsolutePosition;

    private final StatusSignal<Double> driveMotorPosition;
    private final StatusSignal<Double> driveMotorVelocity;
    private final StatusSignal<Double> driveMotorAppliedVolts;
    private final StatusSignal<Double> driveMotorCurrent;
    private final StatusSignal<Integer> driveMotorFaults;
    private final StatusSignal<Double> driveMotorTemp;
    private final StatusSignal<Double> steerMotorPosition;
    private final StatusSignal<Double> steerMotorVelocity;
    private final StatusSignal<Double> steerMotorAppliedVolts;
    private final StatusSignal<Double> steerMotorStatorCurrent;
    private final StatusSignal<Integer> steerMotorFaults;
    private final StatusSignal<Double> steerMotorTemp;

    private final Rotation2d absoluteEncoderOffset;

    public SwerveModuleIoTalonFx(WheelModuleIndex index) {

        // Assign Motor and Encoder Ids and configue wheel offset.
        absoluteEncoderOffset = CONSTANTS.getSwerveModuleEncoderOffsets()[index.value];

        switch (index) {
            case FRONT_LEFT:
                driveMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsFrontLeft().DRIVE_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                steerMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsFrontLeft().STEER_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                cancoder = new CANcoder(CONSTANTS.getSwirveModuleHardwareIdsFrontLeft().CANCODER_ID,
                        CONSTANTS.getCanivoreBusId());
                break;
            case FRONT_RIGHT:
                driveMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsFrontRight().DRIVE_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                steerMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsFrontRight().STEER_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                cancoder = new CANcoder(CONSTANTS.getSwirveModuleHardwareIdsFrontRight().CANCODER_ID,
                        CONSTANTS.getCanivoreBusId());
                break;
            case BACK_LEFT:
                driveMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsBackLeft().DRIVE_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                steerMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsBackLeft().STEER_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                cancoder = new CANcoder(CONSTANTS.getSwirveModuleHardwareIdsBackLeft().CANCODER_ID,
                        CONSTANTS.getCanivoreBusId());
                break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsBackRight().DRIVE_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                steerMotor = new TalonFX(CONSTANTS.getSwirveModuleHardwareIdsBackRight().STEER_MOTOR_ID,
                        CONSTANTS.getCanivoreBusId());
                cancoder = new CANcoder(CONSTANTS.getSwirveModuleHardwareIdsBackRight().CANCODER_ID,
                        CONSTANTS.getCanivoreBusId());
                break;
            default:
                throw new RuntimeException("Invalid module index: " + index);
        }

        // Set Drive TalonFXConfiguration.
        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
        driveTalonFXConfiguration.CurrentLimits = Constants.getDefaultCurrentLimitsConfig();
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        // Set Drive TalonFXConfiguration.
        MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        // Inverted to match our Swerve Drive Module Gear Box & Motors.
        driveMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        driveMotor.getConfigurator().apply(driveMotorOutputConfigs);

        // Set Steer MotorOutputConfigs.
        TalonFXConfiguration steerTalonFXConfiguration = new TalonFXConfiguration();
        steerTalonFXConfiguration.CurrentLimits = Constants.getDefaultCurrentLimitsConfig();
        steerMotor.getConfigurator().apply(steerTalonFXConfiguration);

        // Set Steer MotorOutputConfigs.
        MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        // Inverted to match our Swerve Drive Module Gear Box & Motors.
        steerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        steerMotor.getConfigurator().apply(steerMotorOutputConfigs);

        // Set CAN Coder Configs.
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        cancoder.getConfigurator().apply(canCoderConfiguration);

        // Get current state.
        cancoderAbsolutePosition = cancoder.getAbsolutePosition();

        driveMotorPosition = driveMotor.getPosition();
        driveMotorVelocity = driveMotor.getVelocity();
        driveMotorAppliedVolts = driveMotor.getMotorVoltage();
        driveMotorCurrent = driveMotor.getStatorCurrent();
        driveMotorFaults = driveMotor.getFaultField();
        driveMotorTemp = driveMotor.getDeviceTemp();

        steerMotorPosition = steerMotor.getPosition();
        steerMotorVelocity = steerMotor.getVelocity();
        steerMotorAppliedVolts = steerMotor.getMotorVoltage();
        steerMotorStatorCurrent = steerMotor.getStatorCurrent();
        steerMotorFaults = steerMotor.getFaultField();
        steerMotorTemp = steerMotor.getDeviceTemp();

        // Set Update frequency.
        BaseStatusSignal.setUpdateFrequencyForAll( // Required for odometry, use faster rate
                CONSTANTS.getTalonFxStatusSignalUpdateFrequencyForOdometry(), driveMotorPosition, steerMotorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                CONSTANTS.getTalonFxStatusSignalUpdateFrequencyDefault(),
                driveMotorVelocity,
                driveMotorAppliedVolts,
                driveMotorCurrent,
                cancoderAbsolutePosition,
                steerMotorVelocity,
                steerMotorAppliedVolts,
                steerMotorStatorCurrent);

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

                cancoderAbsolutePosition,

                driveMotorPosition,
                driveMotorVelocity,
                driveMotorAppliedVolts,
                driveMotorCurrent,

                steerMotorPosition,
                steerMotorVelocity,
                steerMotorAppliedVolts,
                steerMotorStatorCurrent,
                driveMotorFaults,
                steerMotorFaults,
                driveMotorTemp,
                steerMotorTemp);

        inputs.cancoderAbsolutePosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
        inputs.cancoderOffsetPosition = inputs.cancoderAbsolutePosition.minus(absoluteEncoderOffset);

        // Inverted driveMotorPosition so that autonomous sees the robot moving in the
        // correct direction.
        inputs.driveMotorPositionRad = Units.rotationsToRadians(-driveMotorPosition.getValueAsDouble())
                / CONSTANTS.getGearRatioOfDriveWheel();
        inputs.driveMotorVelocityRadPerSec = Units.rotationsToRadians(driveMotorVelocity.getValueAsDouble())
                / CONSTANTS.getGearRatioOfDriveWheel();
        inputs.driveMotorAppliedVolts = driveMotorAppliedVolts.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();
        inputs.driveMotorFaults = driveMotorFaults.getValue();
        inputs.driveMotorTemp = driveMotorTemp.getValueAsDouble();

        inputs.steerMotorPosition = Rotation2d
                .fromRotations(steerMotorPosition.getValueAsDouble() / CONSTANTS.getGearRatioOfTurnWheel())
                .plus(Rotation2d.fromRadians(0));
        inputs.steerMotorVelocityRadPerSec = Units.rotationsToRadians(steerMotorVelocity.getValueAsDouble())
                / CONSTANTS.getGearRatioOfTurnWheel();
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorStatorCurrent.getValueAsDouble();
        inputs.steerMotorFaults = steerMotorFaults.getValue();
        inputs.steerMotorTemp = steerMotorTemp.getValueAsDouble();
    }
}