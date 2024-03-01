package frc.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.swerve_module.IndexedSwerveModule.WheelModuleIndex;

public class GameRobotConstants extends AbstractConstants {

    // ==================== Methods (Ctrl + K, Ctrl + 8 to fold regions) =======
    // #region: --------------- Capability Flags -------------------------------
    @Override
    public boolean hasAimerSubsystem() {
        return true;
    }

    @Override
    public boolean hasClimberSubsystem() {
        return true;
    }

    @Override
    public boolean hasNoteSensorSubsystem() {
        return true;
    }

    @Override
    public boolean hasFeederSubsystem() {
        return true;
    }

    @Override
    public boolean hasFlywheelSubsystem() {
        return true;
    }

    @Override
    public boolean hasIntakeSubsystem() {
        return true;
    }

    @Override
    public boolean hasVisionSubsystem() {
        return true;
    }

    @Override
    public boolean hasTraverserSubsystem() {
        return false;
    }

    // #endregion

    // #region: --------------- Driving Configurations -------------------------
    public Measure<Velocity<Angle>> getMaxAngularSpeed() {
        // TODO: Tune.
        return DegreesPerSecond.of(360);
    }

    public Measure<Velocity<Distance>> getMaxLinearSpeed() {
        // TODO: Tune.
        return MetersPerSecond.of(5);
    }

    // #endregion

    // #region: --------------- Hardware ---------------------------------------

    // #region: ----- LEDs -----

    @Override
    public int getLedLength() {
        return 144 /* 1 strip */ * 3;
    }

    // #endregion

    // #region: -------- roboRIO --------

    @Override
    public String getRoboRioSerialNumber() {
        return "03282B9F";
    }

    // #endregion

    // #region: -------- Swerve --------
    @Override
    public Map<WheelModuleIndex, Rotation2d> getSwerveModuleEncoderOffsets() {
        return new HashMap<>() {
            {
                put(WheelModuleIndex.FRONT_LEFT, Rotation2d.fromRadians(0.120));
                put(WheelModuleIndex.FRONT_RIGHT, Rotation2d.fromRadians(-0.023));
                put(WheelModuleIndex.BACK_LEFT, Rotation2d.fromRadians(2.789));
                put(WheelModuleIndex.BACK_RIGHT, Rotation2d.fromRadians(0.853));
            }
        };
    }

    // #endregion

    // #region: --------------- Operation Modes --------------------------------
    @Override
    public boolean isDrivingModeFieldRelative() {
        return true;
    }

    // #endregion

    // #region: --------------- Physical Measurements --------------------------
    @Override
    public double getGearRatioOfDriveWheel() {
        // L3 Gear ratio.
        return (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    }

    @Override
    public double getGearRatioOfTurnWheel() {
        return 12.8;
    }

    @Override
    public Measure<Distance> getWheelDistanceFrontToBack() {
        return Inches.of(23);
    }

    @Override
    public Measure<Distance> getWheelDistanceLeftToRight() {
        return Inches.of(23);
    }

    public Measure<Distance> getWheelRadius() {
        return Inches.of(2);
    }
    // #endregion
}
