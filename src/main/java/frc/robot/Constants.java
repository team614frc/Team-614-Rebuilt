// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;


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
            public static final double FEED_RPM = 5000;

            //Motor Behavior
            public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
            public static final NeutralModeValue NEUTRAL_MODE =NeutralModeValue.Coast;

            //Voltage Limits
            public static final double VELOCITY_VOLTAGE_SLOT = 0.0;
            public static final int NEW_SLOT = 0;
            public static final double VOLTAGE_OUT = 0.0;

            
            //Current Limits
            public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
            public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

            //PID Constants
            public static final int PID_SLOT = 0;
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

             // Feedforward / voltage
            public static final double MAX_VOLTAGE = 12.0;
             public static final double kV = MAX_VOLTAGE / KrakenX60.kFreeSpeed.in(RotationsPerSecond);
             public static final int PERCENT_OUTPUT = 0;



    }

    public static final class FloorConstants {
         //Speed Constants
         public static final double STOP_PERCENT_OUTPUT = 0.0;
        public static final double FLOOR_PERCENT_OUTPUT = 0.83;
        

        //Voltage Limits
        public static final double MAX_VOLTAGE = 12.0;
        public static final double VOLTAGE_OUT = 0.0;

        //Current Limits
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);
    }

    public static final class HangerConstants {

        //Position Constants
        public static final int HOMED = 0; //Inches
        public static final int EXTEND_HOPPER = 2; //Inches
        public static final int HANGING = 6; //Inches
        public static final double HUNG = 0.2; //Inches

        //Motor Behavior
        public static final int MOTOR_ANGLE_DISTANCE = 6;
         public static final int ROTATIONS = 142;
         public static final int EXTENSION_TOLERANCE = 1;

         //Voltage Limits
         public static final int INITIAL_SETPOINT = 0;
         public static final int NEW_SLOT = 0;

         public static final double VOLTAGE_OUT = 0.0;
         public static final double MAX_VOLTAGE = 12.0;

         //Current Limits
         public static final Current STATOR_CURRENT_LIMIT = Amps.of(20);
         public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);

         //PID Constants
            public static final double kP = 10.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;


        //Homing Constants
        public static final double HOMING_VOLTAGE = -0.05;
        public static final double HOMING_CURRENT_THRESHOLD = 0.4;

    }
    public static final class HoodConstants {

        //Position Constants
        public static final int SERVO_LENGTH = 150; 
        public static final int SERVO_SPEED = 20; 
        public static final double MIN_POSITION = 0.01;
        public static final double MAX_POSITION = 0.77;
        public static final double POSITION_TOLERANCE = 0.01;

        public static final double CURRENT_POSITION = 0.5;
        public static final double TARGET_POSITION = 0.5;
        public static final int LAST_UPDATE_TIME = 0;


        //deadband and proportional band constants
        public static final int MAX = 2000;
        public static final int DEADBAND_MAX = 1800;
        public static final int CENTER = 1500;
        public static final int DEADBAND_MIN = 1200;
        public static final int MIN = 1000;
    }

    public static final class  IntakeConstants {
        //Speed Constants
        public static final double STOP_PERCENT_OUTPUT = 0.0;
        public static final double INTAKE_PERCENT_OUTPUT = 0.6;

        //Voltage Limits
        public static final double MAX_VOLTAGE = 12.0;
        public static final double VOLTAGE_OUT = 0.0;
        public static final int PIVOT_VOLTAGE_REQUEST = 0;
        public static final int MOTION_MAGIC_VOLTAGE = 0;
        public static final int NEW_SLOT = 0;
        public static final int ROLLER_VOLTAGE_REQUEST = 0;

        //Position Constants
        public static final int HOMED_ANGLE = 110; //Degrees
        public static final int STOWED_ANGLE = 100; //Degrees
        public static final int INTAKE_ANGLE = -4; 
        public static final int AGITATE = 20; //Degrees

        //Pivot Constants
        public static final double PIVOT_REDUCTION = 50.0; //Degrees
        public static final int POSITION_TOLERANCE = 5;
        public static final double PIVOT_PERCENT_OUTPUT = 0.1;
        //public static final Current PIVOT_HOMING_CURRENT_THRESHOLD = Amp.of(6);


        

        //Current Limits
        public static final int STATOR_CURRENT_LIMIT = 120;
        public static final int SUPPLY_CURRENT_LIMIT = 70;

        //PID Constants
        public static final int KP = 300;
        public static final int KI = 0;
        public static final int KD = 0;

    }
    public static final class  VisionConstants {
    }

    public static final class ShooterConstants {

        //Voltage Limits
        public static final int VELOCITY_TOLERANCE = 100;
        public static final int VELOCITY_VOLTAGE_SLOT = 0;
        public static final int NEW_SLOT = 0;
        public static final int VOLTAGE_OUT = 0;

        public static final int PEAK_REVERSE_VOLTAGE = 0;
        public static final double MAX_VOLTAGE = 12.0;

        //Current Limits
        public static final int STATOR_CURRENT_LIMIT = 120;
        public static final int SUPPLY_CURRENT_LIMIT = 70;

        //RPM Constants
        public static final double DASHBOARD_TARGET_RPM = 0.0;

        //PID Constants
        public static final double kP = 0.5;
        public static final double kI = 2.0;
        public static final double kD = 0.0;

        public static final double STOP_PERCENT_OUTPUT = 0.0;

    }

    public static final class  SwerveConstants {

    }

}
