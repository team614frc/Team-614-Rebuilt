package frc.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Lookup tables for shuttle shots — lobbing balls into the alliance zone. Distance is measured from
 * the robot to the alliance end wall (meters). Tune these values on the actual robot; these are
 * placeholder starting points.
 */
public class ShuttleCalculator {

  // Hood angle interpolation table (distance meters → hood position 0.0–1.0)
  private static final InterpolatingDoubleTreeMap HOOD_TABLE = new InterpolatingDoubleTreeMap();

  // Shooter RPM interpolation table (distance meters → RPM)
  private static final InterpolatingDoubleTreeMap RPM_TABLE = new InterpolatingDoubleTreeMap();

  static {
    // distance (m) → hood position
    // Closer = flatter arc; farther = higher arc
    HOOD_TABLE.put(3.0, 0.72);
    HOOD_TABLE.put(5.0, 0.72);
    HOOD_TABLE.put(7.0, 0.72);
    HOOD_TABLE.put(9.0, 0.72);
    HOOD_TABLE.put(11.0, 0.72);

    // distance (m) → shooter RPM
    RPM_TABLE.put(3.0, 3500.0);
    RPM_TABLE.put(5.0, 3600.0);
    RPM_TABLE.put(7.0, 3700.0);
    RPM_TABLE.put(9.0, 4100.0);
    RPM_TABLE.put(11.0, 4500.0);
  }

  private ShuttleCalculator() {}

  public static double getHoodPosition(double distanceMeters) {
    return HOOD_TABLE.get(distanceMeters);
  }

  public static double getShooterRPM(double distanceMeters) {
    return RPM_TABLE.get(distanceMeters);
  }
}
