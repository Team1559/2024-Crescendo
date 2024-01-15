// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder cancoder;

  private final StatusSignal<Double> driveMotorPosition;
  private final StatusSignal<Double> driveMotorVelocity;
  private final StatusSignal<Double> driveMotorAppliedVolts;
  private final StatusSignal<Double> driveMotorCurrent;

  private final StatusSignal<Double> cancoderAbsolutePosition;
  private final StatusSignal<Double> steerMotorPosition;
  private final StatusSignal<Double> steerMotorVelocity;
  private final StatusSignal<Double> steerMotorAppliedVolts;
  private final StatusSignal<Double> steerMotorStatorCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 12.8;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0: // Front, Left
        driveMotor = new TalonFX(0);
        steerMotor = new TalonFX(1);
        cancoder = new CANcoder(2);
        absoluteEncoderOffset = new Rotation2d(2.26); // MUST BE CALIBRATED
        break;
      case 1: // Front, Right
        driveMotor = new TalonFX(3);
        steerMotor = new TalonFX(4);
        cancoder = new CANcoder(5);
        absoluteEncoderOffset = new Rotation2d(0.35); // MUST BE CALIBRATED
        break;
      case 2: // Back, Left 
        driveMotor = new TalonFX(9);
        steerMotor = new TalonFX(10);
        cancoder = new CANcoder(11);
        absoluteEncoderOffset = new Rotation2d(-3.114); // MUST BE CALIBRATED
        //Flipped by 360 to change the direction the wheels spun.
        break;
      case 3: // Back, Right
        driveMotor = new TalonFX(6);
        steerMotor = new TalonFX(7);
        cancoder = new CANcoder(8);
        absoluteEncoderOffset = new Rotation2d(3.0236); // MUST BE CALIBRATED
        //Flipped by 360 to change the direction the wheels spun.
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(driveMotorConfig);
    setDriveBrakeMode(true);

    var steerMotorConfig = new TalonFXConfiguration();
    steerMotorConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    steerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    steerMotor.getConfigurator().apply(steerMotorConfig);
    setTurnBrakeMode(true);

    CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();
    caNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    //Updated clockwise positive so that the cannon coder and the motor coder had the same polarity. 
    cancoder.getConfigurator().apply(caNcoderConfiguration);

    driveMotorPosition = driveMotor.getPosition();
    driveMotorVelocity = driveMotor.getVelocity();
    driveMotorAppliedVolts = driveMotor.getMotorVoltage();
    driveMotorCurrent = driveMotor.getStatorCurrent();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    steerMotorPosition = steerMotor.getPosition();
    steerMotorVelocity = steerMotor.getVelocity();
    steerMotorAppliedVolts = steerMotor.getMotorVoltage();
    steerMotorStatorCurrent = steerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, driveMotorPosition, steerMotorPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
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
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        driveMotorPosition,
        driveMotorVelocity,
        driveMotorAppliedVolts,
        driveMotorCurrent,
        cancoderAbsolutePosition,
        steerMotorPosition,
        steerMotorVelocity,
        steerMotorAppliedVolts,
        steerMotorStatorCurrent);

    inputs.driveMotorPositionRad =
        Units.rotationsToRadians(driveMotorPosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveMotorVelocityRadPerSec =
        Units.rotationsToRadians(driveMotorVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveMotorAppliedVolts = driveMotorAppliedVolts.getValueAsDouble();
    inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();

    inputs.cancoderAbsolutePosition =
        Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.steerMotorPosition =
        Rotation2d.fromRotations(steerMotorPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.steerMotorVelocityRadPerSec =
        Units.rotationsToRadians(steerMotorVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
    inputs.steerMotorCurrentAmps = steerMotorStatorCurrent.getValueAsDouble();
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
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    steerMotor.getConfigurator().apply(config);
  }
}