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

    public static final double SIGNAL_FREQUENCY = 75;

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
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int REAR_LEFT_CANCODER_ID = 14;
    public static final int REAR_RIGHT_CANCODER_ID = 13;

    public static final double FRONT_LEFT_ZERO_ANGLE = 0.137939453125;
    public static final double FRONT_RIGHT_ZERO_ANGLE = -0.4228515625;
    public static final double REAR_LEFT_ZERO_ANGLE = -0.475341796875;
    public static final double REAR_RIGHT_ZERO_ANGLE = -0.0595703125;

    //inverts may vary
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

    public static final double MAX_SPEED_METERS_PER_SECOND = 6.94;

    public static final double HEADING_ACCEPTABLE_ERROR_DEGREES = 2;
  }
  
  public static final class ModuleConstants { 
    public static final double DRIVE_GEAR_RATIO = 4.59;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS =  WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_TO_METERS_PER_SECOND = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

    public static final double TURN_P = 116.0; 
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.64; 

    public static final double TURN_S = 0.0;
    public static final double TURN_V = 0.0;
    public static final double TURN_A = 0.0;

    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 30; 
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 24;

    public static final double DRIVE_P = 0.4;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    public static final double DRIVE_S = 0.16;
    public static final double DRIVE_V = 0.120303149734; 
    public static final double DRIVE_A = 0.020165577342;
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
      {0, 0.01, 0.05, 10},
      {1.5, 0.01, 0.01, 10},
      {3, 0.15, 1.2, 30},
      {4.5, 1.5, 5.0, 90},
      {6, 3.0, 8.0, 180}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 5},
      {1.5, 0.02, 0.02, 5},
      {3, 0.04, 0.04, 15},
      {4.5, 0.1, 0.1, 30},
      {6, 0.3, 0.3, 60}
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

    public static final Rotation2d RED_AMP_ROTATION = Rotation2d.fromDegrees(90);
    public static final Rotation2d BLUE_AMP_ROTATION = Rotation2d.fromDegrees(90);
    
    public static final double RED_SPEAKER_X = 16.511;
    public static final double RED_SPEAKER_Y = 5.529;

    public static final double BLUE_SPEAKER_X = 0;
    public static final double BLUE_SPEAKER_Y = 5.511;
  }

  public static final class JoystickConstants {
    public static final int DRIVER_LEFT_STICK_X = 0;
    public static final int DRIVER_LEFT_STICK_Y = 1;
    public static final int DRIVER_RIGHT_STICK_X = 4;
    public static final int X_BUTTON = 3;
    public static final int DRIVER_RIGHT_BUMPER_ID = 6;
  }
  
  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 5;
    public static final double INTAKE_MOTOR_SPEED = -1;
  }

  public static final class PivotConstants {
    public static final int LEADER_PIVOT_MOTOR_ID = 9;
    public static final int FOLLOWER_PIVOT_MOTOR_ID = 10;
    public static final int PIVOT_ENCODER_ID = 33;
    public static final double PIVOT_INTAKE_ANGLE = 0.0;

    public static final double PIVOT_P = 0.0;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;
    public static final double PIVOT_G = 0.0;

    public static final double ANGLE_ZERO = 0.0;
    public static final SensorDirectionValue ENCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;

    public static final double SHOOT_AMP_ANGLE = 1-9;
    public static final double PIVOT_ACCEPTABLE_ERROR = 1;

    public static final double PIVOT_NEUTRAL_ANGLE = 0-9;

    public static final double PIVOT_CLIMB_ANGLE = 0-9;

    public static double[][] SPEAKER_PIVOT_POSITION = {
      // Distance, Angle
      {Units.feetToMeters(5.5), 0-9},
      {Units.feetToMeters(7), 0-9},
      {Units.feetToMeters(8.5), 0-9}, 
      {Units.feetToMeters(10), 0-9},
      {Units.feetToMeters(11.5), 0-9},
      {Units.feetToMeters(13), 0-9},
      {Units.feetToMeters(14.5), 0-9},
      {Units.feetToMeters(16), 0-9}
    };
  }

  public static final class ShooterConstants {
    public static final int LEADER_FLYWHEEL_ID = 4;
    public static final int FOLLOWER_FLYWHEEL_ID = 6;
    public static final int ROLLER_MOTOR_ID = 2;

    // public static final int SHOOTER_LIMIT_SWITCH_ID = 0-9;
    // public static final int SHOOTER_NOTE_SENSOR_ID = 0-9;

    public static final double LEFT_SHOOT_SPEAKER_RPM = 0-9;
    public static final double RIGHT_SHOOT_SPEAKER_RPM = 0-9;

    public static final double SHOOT_P = 0.0;
    public static final double SHOOT_I = 0.0;
    public static final double SHOOT_D = 0.0;

    public static final double ROLLER_SPEED = 0.3;
    public static final double SHOOT_AMP_SPEED = 0.3;
    
    public static final double GEAR_RATIO_MOTOR_TO_FLYWHEEL = 30.0 / 40.0;

    public static final double AUTO_SHOOT_P = 0.0;
    public static final double AUTO_SHOOT_I = 0.0;
    public static final double AUTO_SHOOT_D = 0.0;
    public static final double AUTO_SHOOT_S = 0.0;
    public static final double AUTO_SHOOT_V = 0.0;
    public static Constraints AUTO_SHOOT_CONSTRAINTS = new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 5);

    public static double[][] SPEAKER_SHOOT_RPMS = {
      //distance, rpm
      {Units.feetToMeters(5.5), 1770},
      {Units.feetToMeters(7), 1665},
      {Units.feetToMeters(8.5), 1560}, 
      {Units.feetToMeters(10), 1350},
      {Units.feetToMeters(11.5), 1300},
      {Units.feetToMeters(13), 1280},
      {Units.feetToMeters(14.5), 1260},
      {Units.feetToMeters(16), 1300}
   };
  }

  public static final class TrajectoryConstants {
    
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));
    public static final double MAX_SPEED = 5.7;
    public static final double MAX_ACCELERATION = 3.5;
    public static final double REALTIME_TRANSLATION_CONTROLLER_P = .35;
    public static final double REALTIME_THETA_CONTROLLER_P = .8;
    public static final double AUTO_SHOOT_HEADING_OFFSET = 2;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;
    // The length of the field in the x direction (left to right)
    public static final double FIELD_LENGTH_METERS = 16.54175;
    // The length of the field in the y direction (top to bottom)
    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(REALTIME_TRANSLATION_CONTROLLER_P);
    public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(REALTIME_THETA_CONTROLLER_P);
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;

    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
      TRANSLATION_CONSTANTS, 
      ROTATION_CONSTANTS, // Rotation constants 
      DriveConstants.MAX_SPEED_METERS_PER_SECOND, 
      0.4, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }
}

