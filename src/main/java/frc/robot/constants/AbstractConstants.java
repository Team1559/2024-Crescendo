package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class AbstractConstants {

    // ========================= Enums =========================================
    public static enum OperatingMode {
        REAL_WORLD,
        SIMULATION,
        LOG_REPLAY
    }

    // ========================= Static CONSTANTS ==============================
    private static final boolean FORCE_GAME_ROBOT_CONSTANTS = false;
    private static final AbstractConstants GAME_ROBOT_CONSTANTS = new GameRobotConstants();
    private static final AbstractConstants TEST_ROBOT_CONSTANTS = new TestRobotConstants();

    public static final AbstractConstants CONSTANTS = isGameRobot() ? GAME_ROBOT_CONSTANTS : TEST_ROBOT_CONSTANTS;

    // ========================= Static Methods ================================
    public static boolean isGameRobot() {
        String robotSerialNumber = System.getenv("serialnum");

        boolean isGameRobot = FORCE_GAME_ROBOT_CONSTANTS
                || robotSerialNumber.equals(GAME_ROBOT_CONSTANTS.getRoboRioSerialNumber())
                || !robotSerialNumber.equals(TEST_ROBOT_CONSTANTS.getRoboRioSerialNumber());
        System.out.println("Serial number: " + robotSerialNumber);
        System.out.println("isGameRobot: " + isGameRobot);
        return isGameRobot;
    }

    // ========================= Methods =======================================
    // ---------- Operation Modes ----------
    public OperatingMode getCurrentOperatingMode() {
        return OperatingMode.REAL_WORLD;
    }

    public abstract boolean isDrivingModeFieldRelative();

    // ---------- Capabilities Flags --------
    public abstract boolean hasAimerSubsystem();

    public abstract boolean hasColorSensorSubsystem();

    public abstract boolean hasFeederSubsystem();

    public abstract boolean hasFlywheelSubsystem();

    public abstract boolean hasIntakeSubsystem();

    public abstract boolean hasShooterSubsystemGroup();

    public abstract boolean hasVisionSubsystem();

    // ---------- Hardware ----------
    // --- roboRIO ---
    public abstract String getRoboRioSerialNumber();

    // --- Swerve ---
    /**
     * The index of the Rotation matches the Index of the Module in Advaltage Scope.
     * <p>
     * Note: Offsetting by 180 degrees will invert the direction the wheel spins.
     * </p>
     */
    public abstract Rotation2d[] getSwerveModuleEncoderOffsets();
}
