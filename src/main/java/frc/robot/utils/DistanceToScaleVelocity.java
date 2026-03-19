// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class DistanceToScaleVelocity {
    static double base = 4.30326; // based on desmos regression

    public static double distanceToScaleVelocity(Distance distanceToHub) {
        double scaleFactor = Math.log(distanceToHub.in(Meters) + 1) / Math.log(base);
        scaleFactor = scaleFactor > 1 ? 1 : scaleFactor;
        scaleFactor = scaleFactor < 1 ? 0 : scaleFactor;
        return scaleFactor;
    }

    // regression is based on these values, this map is for reference (not being used in code)
    public Map<Distance, Double> distanceToScalar = Map.ofEntries(
            Map.entry(Meters.of(1.0), 0.3),
            Map.entry(Meters.of(2.0), 0.8),
            Map.entry(Meters.of(3.0), 1.0));
}
