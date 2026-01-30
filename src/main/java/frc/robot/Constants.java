// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

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

  public static final class FeederConstants {
    public static final AngularVelocity FEED_SPEED = Revolutions.per(Minute).of(5000);

    // Motor Behavior
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    // Voltage Limits
    public static final Voltage VELOCITY_VOLTAGE_SLOT = Volts.of(0.0);
    public static final int NEW_SLOT = 0;
    public static final Voltage VOLTAGE_OUT = Volts.of(0.0);

    // Current Limits
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

    // PID Constants
    public static final int PID_SLOT = 0;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Feedforward / voltage
    public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
    public static final double kV =
        MAX_VOLTAGE.in(Volts) / KrakenX60.kFreeSpeed.in(RotationsPerSecond);
    public static final int PERCENT_OUTPUT = 0;
  }

  public static final class FloorConstants {
    // Speed Constants
    public static final double STOP_PERCENT_OUTPUT = 0.0;
    public static final double FLOOR_PERCENT_OUTPUT = 0.83;

    // Voltage Limits
    public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
    public static final Voltage VOLTAGE_OUT = Volts.of(0.0);

    // Current Limits
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
  }

  public static final class HangerConstants {

    // Position Constants
    public static final int HOMED = 0; // Inches
    public static final int EXTEND_HOPPER = 2; // Inches
    public static final int HANGING = 6; // Inches
    public static final double HUNG = 0.2; // Inches

    // Voltage Limits
    public static final int INITIAL_SETPOINT = 0;
    public static final int NEW_SLOT = 0;

    public static final Voltage VOLTAGE_OUT = Volts.of(0.0);
    public static final Voltage MAX_VOLTAGE = Volts.of(12.0);

    // Current Limits
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(20);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);

    // PID Constants
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Homing Constants
    public static final Voltage HOMING_VOLTAGE = Volts.of(-0.05);
    public static final double HOMING_CURRENT_THRESHOLD = 0.4;

    public static Per<DistanceUnit, AngleUnit> HANGER_EXTENSION_PER_MOTOR_ANGLE =
        Inches.of(6).div(Rotations.of(142));
    public static Distance EXTENSION_TOLERANCE = Inches.of(1);
  }

  public static final class HoodConstants {

    // Position Constants
    public static final int SERVO_LENGTH = 150;
    public static final int SERVO_SPEED = 20;
    public static final double MIN_POSITION = 0.01;
    public static final double MAX_POSITION = 0.77;
    public static final double POSITION_TOLERANCE = 0.01;

    public static final double CURRENT_POSITION = 0.5;
    public static final double TARGET_POSITION = 0.5;
    public static final int LAST_UPDATE_TIME = 0;

    // deadband and proportional band constants
    public static final int MAX = 2000;
    public static final int DEADBAND_MAX = 1800;
    public static final int CENTER = 1500;
    public static final int DEADBAND_MIN = 1200;
    public static final int MIN = 1000;

    public static final LinearVelocity MAX_SERVO_SPEED =
        Millimeters.of(HoodConstants.SERVO_SPEED).per(Second);
  }

  public static final class IntakeConstants {
    // Speed Constants
    public static final double STOP_PERCENT_OUTPUT = 0.0;
    public static final double INTAKE_PERCENT_OUTPUT = 0.6;

    // Voltage Limits
    public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
    public static final Voltage VOLTAGE_OUT = Volts.of(0.0);
    public static final Voltage PIVOT_VOLTAGE_REQUEST = Volts.of(0);
    public static final Voltage MOTION_MAGIC_VOLTAGE = Volts.of(0);
    public static final int NEW_SLOT = 0;
    public static final Voltage ROLLER_VOLTAGE_REQUEST = Volts.of(0);

    // Position Constants
    public static final Angle HOMED_ANGLE = Degree.of(110);
    public static final Angle STOWED_ANGLE = Degree.of(100);
    public static final Angle INTAKE_ANGLE = Degree.of(-4);
    public static final Angle AGITATE = Degree.of(20);

    // Pivot Constants
    public static final double PIVOT_REDUCTION = 50.0; // Degrees
    public static final int POSITION_TOLERANCE = 5;
    public static final double PIVOT_PERCENT_OUTPUT = 0.1;
    // public static final Current PIVOT_HOMING_CURRENT_THRESHOLD = Amp.of(6);

    // Current Limits
    public static final int STATOR_CURRENT_LIMIT = 120;
    public static final int SUPPLY_CURRENT_LIMIT = 70;

    // PID Constants
    public static final int KP = 300;
    public static final int KI = 0;
    public static final int KD = 0;
  }

  public static final class ShooterConstants {

    // Voltage Limits
    public static final int VELOCITY_TOLERANCE = 100;
    public static final int VELOCITY_VOLTAGE_SLOT = 0;
    public static final int NEW_SLOT = 0;
    public static final Voltage VOLTAGE_OUT = Volts.of(0);

    public static final Voltage PEAK_REVERSE_VOLTAGE = Volts.of(0);
    public static final Voltage MAX_VOLTAGE = Volts.of(12.0);

    // Current Limits
    public static final int STATOR_CURRENT_LIMIT = 120;
    public static final int SUPPLY_CURRENT_LIMIT = 70;

    // RPM Constants
    public static final double DASHBOARD_TARGET_RPM = 0.0;

    // PID Constants
    public static final double kP = 0.5;
    public static final double kI = 2.0;
    public static final double kD = 0.0;

    public static final double STOP_PERCENT_OUTPUT = 0.0;
  }

  public static final class SwerveConstants {}
}
