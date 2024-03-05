package frc.robot.io.swerve_module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIoSim implements SwerveModuleIo {

    private static final double LOOP_PERIOD_SECS = 0.02;
    private static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final Rotation2d cancoderOffset = new Rotation2d(Math.random() * 2.0 * Math.PI);

    private Measure<Voltage> driveVoltsTarget = Volts.zero();
    private Measure<Voltage> steerVoltsTarget = Volts.zero();

    public SwerveModuleIoSim(DCMotor driveMotorSim, DCMotor turnMotorSim) {
        // TODO: Configure `gearing` & `jKgMetersSquared`.
        driveSim = new DCMotorSim(driveMotorSim, 6.75, 0.025);
        turnSim = new DCMotorSim(turnMotorSim, 150.0 / 7.0, 0.004);
    }

    // ========================= Functions =========================
    @Override
    public Measure<Temperature> getMaxSafeMotorTemperature() {
        return Celsius.of(Double.MAX_VALUE);
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        driveVoltsTarget = clampVolts(volts);
        driveSim.setInputVoltage(driveVoltsTarget.in(Volts));
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> volts) {
        steerVoltsTarget = clampVolts(volts);
        turnSim.setInputVoltage(steerVoltsTarget.in(Volts));
    }

    private static Measure<Voltage> clampVolts(Measure<Voltage> volts) {
        return Volts.of(MathUtil.clamp(volts.in(Volts), MAX_VOLTAGE.negate().in(Volts), MAX_VOLTAGE.in(Volts)));
    }

    @Override
    public void updateInputs(SwerveModuleIoInputs inputs) {

        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        // ---------- CANCoder ----------
        inputs.cancoderOffsetPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.cancoderAbsolutePosition = inputs.cancoderAbsolutePosition.plus(cancoderOffset);

        // ---------- Drive Motor ----------
        inputs.driveMotorCurrentActual = Amps.of(driveSim.getCurrentDrawAmps());
        inputs.driveMotorPositionAbsolute = Rotation2d.fromRotations(driveSim.getAngularPositionRotations());
        inputs.driveMotorVoltsTarget = driveVoltsTarget;
        inputs.driveMotorVelocityActual = RotationsPerSecond.of(driveSim.getAngularPositionRotations());

        // ---------- Steer Motor ----------
        inputs.steerMotorCurrentActual = Amps.of(turnSim.getCurrentDrawAmps());
        inputs.steerMotorPositionAbsolute = Rotation2d.fromRotations(turnSim.getAngularPositionRotations());
        inputs.steerMotorVoltsTarget = steerVoltsTarget;
        inputs.steerMotorVelocityActual = RotationsPerSecond.of(turnSim.getAngularPositionRotations());
    }
}