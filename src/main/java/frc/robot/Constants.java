/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                 */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class HardwareConstants {
    public static final double TIMEOUT_S = 0.05;

    public static final double SIGNAL_FREQUENCY = 250;

    public static final String CANIVORE_CAN_BUS_STRING = "canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    public static final double MIN_FALCON_DEADBAND = 0.0001;

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

  }

  public static final class DriveConstants {
    public static final double X_POS_TRUST = 0.03; // Meters
    public static final double Y_POS_TRUST = 0.03; // Meters
    public static final double ANGLE_TRUST = Units.degreesToRadians(1); // Radians

    // Wheel base and track width are measured by the center of the swerve modules, not the frame of the robot
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(26);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
    );

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 22;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 24;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 23;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 21;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 8;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 7;

    public static final int FRONT_LEFT_CANCODER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 0;
    public static final int REAR_LEFT_CANCODER_ID = 14;
    public static final int REAR_RIGHT_CANCODER_ID = 13;

    public static final double FRONT_LEFT_ZERO_ANGLE = 0.137939453125;
    public static final double FRONT_RIGHT_ZERO_ANGLE = -0.420654296875;
    public static final double REAR_LEFT_ZERO_ANGLE = -0.475341796875;
    public static final double REAR_RIGHT_ZERO_ANGLE = -0.0595703125;

    public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    
    public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED = InvertedValue.Clockwise_Positive;

    public static final InvertedValue FRONT_LEFT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue FRONT_RIGHT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive; 
    public static final InvertedValue REAR_LEFT_DRIVE_ENCODER_REVERSED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue REAR_RIGHT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive;
    
    // TODO: tune
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 5;

    public static final double MAX_SPEED_METERS_PER_SECOND = 6.95;

    public static final double HEADING_ACCEPTABLE_ERROR_RADIANS = Units.degreesToRadians(2);
  }
  
  public static final class ModuleConstants { 
    public static final double DRIVE_GEAR_RATIO = 4.59;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS =  WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_TO_METERS_PER_SECOND = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

    //TODO: Test these:
    public static final double DRIVE_SUPPLY_LIMIT = 40.0;
    public static final double DRIVE_STATOR_LIMIT = 55.0;  // TODO: Do we need this? -Z

    public static final double TURN_P = 116.0; 
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.64; 

    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 30; 
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 24;

    public static final double DRIVE_P = 0.417;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    public static final double DRIVE_S = 0.16;
    // These values were gotten using recalc, then converted to the correct units & were confirmed through testing and characterization
    // https://www.reca.lc/drive?appliedVoltageRamp=%7B%22s%22%3A1200%2C%22u%22%3A%22V%2Fs%22%7D&batteryAmpHours=%7B%22s%22%3A18%2C%22u%22%3A%22A%2Ah%22%7D&batteryResistance=%7B%22s%22%3A0.018%2C%22u%22%3A%22Ohm%22%7D&batteryVoltageAtRest=%7B%22s%22%3A12.6%2C%22u%22%3A%22V%22%7D&efficiency=97&filtering=1&gearRatioMax=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&gearRatioMin=%7B%22magnitude%22%3A3%2C%22ratioType%22%3A%22Reduction%22%7D&maxSimulationTime=%7B%22s%22%3A4%2C%22u%22%3A%22s%22%7D&maxSpeedAccelerationThreshold=%7B%22s%22%3A0.15%2C%22u%22%3A%22ft%2Fs2%22%7D&motor=%7B%22quantity%22%3A4%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&motorCurrentLimit=%7B%22s%22%3A60%2C%22u%22%3A%22A%22%7D&numCyclesPerMatch=24&peakBatteryDischarge=20&ratio=%7B%22magnitude%22%3A4.59%2C%22ratioType%22%3A%22Reduction%22%7D&sprintDistance=%7B%22s%22%3A25%2C%22u%22%3A%22ft%22%7D&swerve=1&targetTimeToGoal=%7B%22s%22%3A2%2C%22u%22%3A%22s%22%7D&throttleResponseMax=0.99&throttleResponseMin=0.5&weightAuxilliary=%7B%22s%22%3A24%2C%22u%22%3A%22lbs%22%7D&weightDistributionFrontBack=0.5&weightDistributionLeftRight=0.5&weightInspected=%7B%22s%22%3A125%2C%22u%22%3A%22lbs%22%7D&wheelBaseLength=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D&wheelBaseWidth=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&wheelCOFDynamic=0.9&wheelCOFLateral=1.1&wheelCOFStatic=1.1&wheelDiameter=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D
    public static final double DRIVE_V = 1.73 * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO; // = 0.1203 V*s/m 
    public static final double DRIVE_A = 0.32 * WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO; // = 0.02225 V*s^2/m
  }

  public static final class VisionConstants {

    public static final double VISION_X_POS_TRUST = 0.5; // meters
    public static final double VISION_Y_POS_TRUST = 0.5; // meters
    public static final double VISION_ANGLE_TRUST = Units.degreesToRadians(50); // radians
  
    public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 2;
    public static final double FOV_MARGIN_OF_ERROR = 27;
  
    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    public static final String BACK_LIMELIGHT_NAME = "limelight-back";

    public static final double[][] APRIL_TAG_POSITIONS = {
      // {x, y, z, rotation (degrees)}
      {Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), 120}, // 1
      {Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), 120}, // 2
      {Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), 180}, // 3
      {Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), 180}, // 4
      {Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38), 270}, // 5
      {Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38), 270}, // 6
      {-Units.inchesToMeters(1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), 0}, // 7
      {-Units.inchesToMeters(1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), 0}, // 8
      {Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), 60}, // 9
      {Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), 60}, // 10
      {Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0), 300}, // 11
      {Units.inchesToMeters(468.69), Units.inchesToMeters(177.1), Units.inchesToMeters(52.0), 60}, // 12
      {Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0), 180}, // 13
      {Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0), 0}, // 14
      {Units.inchesToMeters(182.73), Units.inchesToMeters(177.1), Units.inchesToMeters(52.0), 120}, // 15
      {Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0), 240}, // 16
    };

    // TODO: Actually tune/do math for these
    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.02, 0.02, Units.degreesToRadians(180)}, // 2
      {1.5, 0.05, 0.05, Units.degreesToRadians(180)}, // 5
      {3, 1.2, 1.2, Units.degreesToRadians(180)}, // 25
      {4.5, 5.5, 5.5, Units.degreesToRadians(180)}, // 90
      {8, 10.0, 10.0, Units.degreesToRadians(180)} // 180
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, Units.degreesToRadians(180)}, // 0.5
      {1.5, 0.03, 0.03, Units.degreesToRadians(180)}, // 0.7
      {3, 0.06, 0.06, Units.degreesToRadians(180)}, // 4
      {4.5, 0.15, 0.15, Units.degreesToRadians(180)}, // 30
      {8, 1.0, 1.0, Units.degreesToRadians(180)} // 90
    };
  }

  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(653);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(325);

    // TODO: in meters from the bottom left of the field
    // TODO: add point for in front of each:
    public static final double RED_AMP_X = 14.613;
    public static final double RED_AMP_Y = 8.197;

    public static final double RED_AMP_SHOOT_X = 14.613;
    public static final double RED_AMP_SHOOT_Y = 7.767;

    public static final double BLUE_AMP_X = 1.9;
    public static final double BLUE_AMP_Y = 8.161;
    public static final double BLUE_AMP_SHOOT_X = 1.9;
    public static final double BLUE_AMP_SHOOT_Y = 7.767;

    public static final Rotation2d RED_AMP_ROTATION = Rotation2d.fromDegrees(-90);
    public static final Rotation2d BLUE_AMP_ROTATION = Rotation2d.fromDegrees(-90);
    
    public static final double RED_SPEAKER_X = 16.511;
    public static final double RED_SPEAKER_Y = 5.55;

    public static final double BLUE_SPEAKER_X = 0;
    public static final double BLUE_SPEAKER_Y = 5.55;

    public static final double RED_LOADING_STATION_X = 1.1;
    public static final double RED_LOADING_STATION_Y = 1.169;

    public static final double BLUE_LOADING_STATION_X = 15.41;
    public static final double BLUE_LOADING_STATION_Y = 1.13;
  }
  
  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 16;
    public static final double INTAKE_SPEED = 0.7;

    public static final int OTB_PIVOT_ID = 50;
    public static final int OTB_INTAKE_ID = 40;
    public static final double OTB_ROTOR_OFFSET = -0.3095703125;

    public static final double MM_ACCELERATION = 4;
    public static final double MM_VELOCITY = 10;

    public static final double INTAKE_P = 0;
    public static final double INTAKE_I = 0;
    public static final double INTAKE_D = 0;
    public static final double INTAKE_G = 0;

    public static final double MAX_OVERTHEBUMPER_ANGLE = 0-9;
    public static final double MIN_OVERTHEBUMPER_ANGLE = 0-9;

    public static final double INTAKE_OVERTHEBUMPER_SPEED = 1;

    public static final double INTAKE_PIVOT_ACCEPTABLE_ERROR = 0.01;
  }

  public static final class PivotConstants {
    public static final int LEADER_PIVOT_MOTOR_ID = 9;
    public static final int FOLLOWER_PIVOT_MOTOR_ID = 10;
    public static final int PIVOT_ENCODER_ID = 33;

    public static final double MIN_ANGLE = -0.015080078125;
    public static final double MAX_ANGLE = 0.37158203125;

    public static final double PIVOT_INTAKE_ANGLE = -0.006591796875;

    public static final double PIVOT_P = 180.0;
    public static final double PIVOT_I = 0.0; //60.0 
    public static final double PIVOT_D = 0.0;
    public static final double PIVOT_G = 1.7320;

    public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 4;
    public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 10;

    public static final double PIVOT_NEUTRAL_SPEED = 0;

    public static final double ANGLE_ZERO = -0.026123046875;//-0.00732421875;//-0.0146484375;
    public static final SensorDirectionValue ENCODER_REVERSED = SensorDirectionValue.Clockwise_Positive;

    public static final double SHOOT_AMP_ANGLE = 0.182392578125;
    public static final double PIVOT_ACCEPTABLE_ERROR = 0.015;

    public static final double PIVOT_NEUTRAL_ANGLE = 0;

    //TODO: get these!!!!!11
    public static final double PIVOT_START_CLIMB_ANGLE = 0;
    public static final double PIVOT_END_CLIMB_ANGLE = 0;

    public static double[][] SPEAKER_PIVOT_POSITION = {
      // Distance, Angle (rotations)
      {1.37, 0.0},
      {1.6, 0.021},
      {1.8, 0.028},
      {2.0, 0.04},
      {2.2, 0.042},
      {2.4, 0.045},
      {2.6, 0.0485},
      {2.7, 0.0515},
      {2.8, 0.056},
      {3.0, 0.061},
      {3.2, 0.063},
      {3.4, 0.0666},
      {3.6, 0.0697},
      {3.8, 0.074},
      {4.0, 0.077},
    };
  }

  public static final class ShooterConstants {
    public static final int LEADER_FLYWHEEL_ID = 4;
    public static final int FOLLOWER_FLYWHEEL_ID = 12;
    public static final int ROLLER_MOTOR_ID = 2;

    public static final double SHOOTER_SUPPLY_LIMIT = 60;
    public static final double SHOOTER_STATOR_LIMIT = 60;
    public static final boolean SHOOTER_STATOR_ENABLE = true;
    public static final boolean SHOOTER_SUPPLY_ENABLE = true;

    public static final double ROLLER_NEUTRAL_SPEED = 0;
    public static final double SHOOTER_NEUTRAL_SPEED = 0;

    public static final int SHOOTER_NOTE_SENSOR_ID = 0;

    public static final double SHOOT_SPEAKER_RPM = 4000;

    public static final int SHOOTER_ACCEPTABLE_RPM_ERROR = 50;

    public static final double SHOOT_P = 80.0;
    public static final double SHOOT_I = 0.0;
    public static final double SHOOT_D = 0.0;
    public static final double SHOOT_S = 0.36;
    public static final double SHOOT_V = 0.12287;
    public static final double SHOOT_A = 0.00520;

    public static final double ROLLER_SHOOT_SPEED = 1;
    public static final double ROLLER_INTAKE_SPEED = 0.2;
    public static final double SHOOT_AMP_RPM = 2000;
    
    public static final double AUTO_SHOOT_P = 4.5;
    public static final double AUTO_SHOOT_I = 0.0;
    public static final double AUTO_SHOOT_D = 0.0;
    public static Constraints AUTO_SHOOT_CONSTRAINTS = new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2);
  }

  public static final class TrajectoryConstants {

    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));
    public static final double MAX_SPEED = 5.0;
    public static final double MAX_ACCELERATION = 1;

    public static final double AUTO_TRANSLATION_P = 9; // 6.5
    public static final double AUTO_THETA_P = 8;
    public static final double AUTO_SHOOT_HEADING_OFFSET = 2; 

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;

    public static final PIDConstants REALTIME_TRANSLATION_PID = new PIDConstants(0.2, 0, 0);
    public static final PIDConstants REALTIME_THETA_PID = new PIDConstants(0.1, 0, 0 );
    public static final HolonomicPathFollowerConfig CONFIG = new HolonomicPathFollowerConfig(REALTIME_TRANSLATION_PID, REALTIME_THETA_PID, MAX_SPEED, 0.876, new ReplanningConfig());

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;

  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final int LEFT_STICK_X_ID = 0;
    public static final int LEFT_STICK_Y_ID = 1;
    public static final int RIGHT_STICK_X_ID = 4;

    public static final int A_BUTTON_ID = 1;
    public static final int B_BUTTON_ID = 2;
    public static final int X_BUTTON_ID = 3;
    public static final int Y_BUTTON_ID = 4;

    public static final int LEFT_BUMPER_ID = 5;
    public static final int RIGHT_BUMPER_ID = 6;
    public static final int RIGHT_D_PAD_ID = 90;
    public static final int LEFT_TRIGGER_ID = 2;
    public static final int RIGHT_TRIGGER_ID = 3;
    public static final int RIGHT_STICK_Y_ID = 5;

  }

  
  public static final class LEDConstants {
    public static final int LED_PORT = 0;

    public static final class SparkConstants {
      // This subclass contains the constant values for the LED patterns.
      public static final double RAINBOW = -0.99;
      public static final double SHOT_RED = -0.85;
      public static final double SHOT_BLUE = -0.83;
      public static final double SHOT_WHITE = -0.81;
      public static final double RED_ALLIANCE_BLINKIN = -0.39;
      public static final double RAINBOW_WAVE = -0.45;
      public static final double OCEAN = -0.41;
      public static final double BOUNCE_RED = -0.35;
      public static final double BOUNCE_GRAY = -0.33;
      public static final double HEARTBEAT_RED = -0.25;
      public static final double HEARTBEAT_GRAY = -0.19;
      public static final double STROBE_RED = -0.11;
      public static final double STROBE_BLUE = -0.09;
      public static final double STROBE_GOLD = -0.07;
      public static final double STROBE_WHITE = -0.05;

      public static final double HEARTBEAT_1 = 0.43;
      public static final double HEARTBEAT_2 = 0.27;

      public static final double MAGENTA = 0.57;
      public static final double DARK_RED = 0.59;
      public static final double RED = 0.61;
      public static final double VERMILION = 0.63;
      public static final double ORANGE = 0.65;
      public static final double GOLD = 0.67;
      public static final double YELLOW = 0.69;
      public static final double LAWN_GREEN = 0.71;
      public static final double LIME = 0.73;
      public static final double DARK_GREEN = 0.75;
      public static final double GREEN = 0.77;
      public static final double CYAN = 0.79;
      public static final double AQUA = 0.81;
      public static final double SKY_BLUE = 0.83;
      public static final double DARK_BLUE = 0.85;
      public static final double BLUE = 0.87;
      public static final double INDIGO = 0.89;
      public static final double PURPLE = 0.91;
      public static final double WHITE = 0.93;
      public static final double GRAY = 0.95;
      public static final double DARK_GRAY = 0.97;
      public static final double BLACK = 0.99;    
    }
    
    public enum LEDProcess {
      /** alliance color */
      ALLIANCE_COLOR (0, 0, 0, 0),
      /** default */
      DEFAULT (0, 0, 0, 0),
      RAINBOW (SparkConstants.RAINBOW, 0, 0, 0),
      RED_ALLIANCE (SparkConstants.RED_ALLIANCE_BLINKIN, 255, 0, 0),
      BLUE_ALLIANCE (SparkConstants.OCEAN, 0, 0, 255),
      SHOOT(SparkConstants.WHITE, 0, 0, 255),
      OFF (SparkConstants.BLACK, 0, 0, 0),
      AUTONOMOUS (SparkConstants.SHOT_WHITE, 0, 0, 0),
      REVERSE_INTAKE (SparkConstants.RED, 0, 255, 0),
      FINISH_LINE_UP (SparkConstants.GREEN, 255, 255, 255),
      GREEN (SparkConstants.GREEN, 0, 255, 0),
      RED (SparkConstants.RED, 255, 0, 0),
      INTAKE (SparkConstants.RED, 255, 0, 0),
      NOTE_IN (SparkConstants.GREEN, 0, 255, 0);

      public final double sparkValue;
      private final int red, green, blue;
      LEDProcess(double sparkValue, int red, int green, int blue) {
        this.sparkValue = sparkValue;
        this.red = red;
        this.green = green;
        this.blue = blue;
      }
    }
  }
}

