package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MathUtils {

    public static Rotation2d clamp(Rotation2d rotation, Rotation2d minRotation, Rotation2d maxRotation) {
        return minRotation != null || maxRotation != null
                ? Rotation2d.fromRotations(MathUtil.clamp(rotation.getRotations(),
                        minRotation == null ? Double.NEGATIVE_INFINITY : minRotation.getRotations(),
                        maxRotation == null ? Double.POSITIVE_INFINITY : maxRotation.getRotations()))
                : rotation;
    }

    public static <U extends Unit<U>> Measure<U> clamp(Measure<U> measure, Measure<U> maxMeasure) {
        return maxMeasure == null ? measure
                : measure.unit().ofBaseUnits(MathUtil.clamp(measure.baseUnitMagnitude(),
                        -Math.abs(maxMeasure.baseUnitMagnitude()), Math.abs(maxMeasure.baseUnitMagnitude())));
    }
}
