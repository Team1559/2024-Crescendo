package org.victorrobotics.frc.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MathUtils {

    public static <U extends Unit<U>> Measure<U> average(Measure<U> measure1, Measure<U> measure2) {
        if (measure1 == measure2) {
            return measure1;
        } else if (measure1 == null) {
            return measure2.divide(2);
        } else if (measure2 == null) {
            return measure1.divide(2);
        } else {
            return measure1.unit().ofBaseUnits((measure1.baseUnitMagnitude() + measure2.baseUnitMagnitude()) / 2);
        }
    }

    public static Rotation2d average(Rotation2d rotation1, Rotation2d rotation2) {
        if (rotation1 == rotation2) {
            return rotation1;
        } else if (rotation1 == null) {
            return rotation2.div(2);
        } else if (rotation2 == null) {
            return rotation1.div(2);
        } else {
            return rotation1.plus(rotation2).div(2);
        }
    }

    public static <U extends Unit<U>> Measure<U> clamp(Measure<U> measure, Measure<U> maxMeasure) {
        return maxMeasure == null ? measure
                : measure.unit().ofBaseUnits(MathUtil.clamp(measure.baseUnitMagnitude(),
                        -Math.abs(maxMeasure.baseUnitMagnitude()), Math.abs(maxMeasure.baseUnitMagnitude())));
    }

    public static Rotation2d clamp(Rotation2d rotation, Rotation2d minRotation, Rotation2d maxRotation) {
        return minRotation != null || maxRotation != null
                ? Rotation2d.fromRotations(MathUtil.clamp(rotation.getRotations(),
                        minRotation == null ? Double.NEGATIVE_INFINITY : minRotation.getRotations(),
                        maxRotation == null ? Double.POSITIVE_INFINITY : maxRotation.getRotations()))
                : rotation;
    }

    public static <U extends Unit<U>> Measure<U> difference(Measure<U> minuend, Measure<U> subtrahend) {
        if (minuend == null && subtrahend == null) {
            return null;
        } else if (minuend != null && subtrahend != null) {
            return minuend.minus(subtrahend);
        } else if (minuend != null) {
            return minuend;
        } else {
            return subtrahend.negate();
        }
    }

    public static Rotation2d difference(Rotation2d minuend, Rotation2d subtrahend) {
        if (minuend == null && subtrahend == null) {
            return null;
        } else if (minuend != null && subtrahend != null) {
            return minuend.minus(subtrahend);
        } else if (minuend != null) {
            return minuend;
        } else {
            return Rotation2d.fromRotations(0).minus(subtrahend);
        }
    }
}
