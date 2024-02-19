package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose.
 * All constants should be declared globally (i.e. public static). Do not put
 * anything
 * functional in this class.
 */
@SuppressWarnings("unused") // < -- Never do this (Gets rid of dead code warnings)
public final class Constants {

    // ========================= CONSTANTS ======================================
    public static final double ADVANTAGE_ODOMETRY_LOG_FREQUENCY = 100.0;
    public static final double ADVANTAGE_DEFAULT_LOG_FREQUENCY = 50.0;

    // ---------- Power Constants ----------
    public static final int NEO_SPARK_BRUSHLESS_CURRENT_LIMIT = 24;
    public static final int NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT = 80;

    // ========================= Configuration Objects ========================
    /**
     * Allow 40A continuous, 80A momentary supply current. See: <a href=
     * "https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html">CTR
     * Electronics: CurrentLimitsConfigs</a>
     * 
     * @return A {@link CurrentLimitsConfigs} object with the default Current
     *         limits.
     */
    public static CurrentLimitsConfigs getDefaultCurrentLimitsConfig() {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40.0;
        limits.SupplyCurrentThreshold = 80.0;
        limits.SupplyTimeThreshold = 0.5;
        return limits;
    }

    // ========================= Constructors ===================================
    /** Makes this class non-instantiable. */
    private Constants() {
    }
}
