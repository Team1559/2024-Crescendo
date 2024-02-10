package frc.robot.util;

import java.util.function.BooleanSupplier;

public class SupplierUtil {
    /**
     * Invert boolean supplier
     * 
     * @param booleanSupplier Supplier to be inverted
     * @return inverted Supplier
     */
    public static BooleanSupplier not(BooleanSupplier booleanSupplier) {
        return () -> !booleanSupplier.getAsBoolean();
    }
}
