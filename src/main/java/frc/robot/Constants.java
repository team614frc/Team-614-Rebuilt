// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5);
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.1;
  }

  public static final class VisionConstants {
    // Gains for alignment
    public static final double ROTATION_KP = 1.75;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 5; // 3.75
    public static final double MIN_ANGULAR_SPEED_RAD_PER_SEC = 0.35;
    public static final double ANGLE_TOLERANCE_DEGREES = 2.75; // 3
    public static final double MIN_VISION_FUSE_PERIOD = 0.06; // ~16 Hz
    public static double lastVisionFuseTime = 0.0;
  }

  public static class KrakenX60 {
    public static final AngularVelocity kFreeSpeed = RPM.of(6000);
  }

 

}
