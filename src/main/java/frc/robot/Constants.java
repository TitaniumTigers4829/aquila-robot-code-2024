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
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
    public static final double TIMEOUT_S = 0.03;
    public static final double TIMEOUT_MS = 30;

    public static final String CANIVORE_CAN_BUS_STRING = "canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    public static final double MIN_FALCON_DEADBAND = 0.0001;

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;


  }

  public static final class DriveConstants {
    public static final double X_POS_TRUST = 0-9; //meters
    public static final double Y_POS_TRUST = 0-9; //meters
    public static final double ANGLE_TRUST = 0-9; //radians

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(0-9);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(0-9);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
    );

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 7;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int REAR_LEFT_CANCODER_ID = 13;
    public static final int REAR_RIGHT_CANCODER_ID = 14;

    public static final double FRONT_LEFT_ZERO_ANGLE = 0-9;
    public static final double FRONT_RIGHT_ZERO_ANGLE = 0-9;
    public static final double REAR_LEFT_ZERO_ANGLE = 0-9;
    public static final double REAR_RIGHT_ZERO_ANGLE = 0-9;

    //inverts may vary
    public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    
    public static final InvertedValue FRONT_LEFT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue FRONT_RIGHT_DRIVE_ENCODER_REVERSED = InvertedValue.Clockwise_Positive; 
    public static final InvertedValue REAR_LEFT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue REAR_RIGHT_DRIVE_ENCODER_REVERSED = InvertedValue.Clockwise_Positive;
    
    
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 5;

    public static final double MAX_SPEED_METERS_PER_SECOND = 5.7;
  }
  
  public static final class ModuleConstants { 
    public static final double DRIVE_GEAR_RATIO = 5.6;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS =  WHEEL_CIRCUMFERENCE_METERS / (DRIVE_GEAR_RATIO);
    public static final double DRIVE_TO_METERS_PER_SECOND = (WHEEL_CIRCUMFERENCE_METERS) / (DRIVE_GEAR_RATIO);

    public static final double TURN_P = 0-9; 
    public static final double TURN_I = 0-9;
    public static final double TURN_D = 0-9;

    public static final double TURN_S = 0-9;
    public static final double TURN_V = 0-9;
    public static final double TURN_A = 0-9;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 0-9; 
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 0-9;
    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
      );

    public static final double DRIVE_P = 0-9;
    public static final double DRIVE_I = 0-9;
    public static final double DRIVE_D = 0-9;

    public static final double DRIVE_S = 0-9;
    public static final double DRIVE_V = 0-9;
    public static final double DRIVE_A = 0-9;
  }

  public static final class VisionConstants {

    public static final double VISION_X_POS_TRUST = 0-9; //meters
    public static final double VISION_Y_POS_TRUST = 0-9; //meters
    public static final double VISION_ANGLE_TRUST = 0-9; //radians
  
    public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 5;
  
    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    public static final String BACK_LIMELIGHT_NAME = "limelight-back";

    public static final double[][] APRIL_TAG_POSITIONS = {
      // { x, y, z}
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 1
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 2
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 3
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 4
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 5
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 6
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 7
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)} // 8
    };

    public static final double[][] CAMERA_CROP_LOOKUP_TABLE = {
      // TODO: All of these are placeholder values
      // {x position in meters, limelight lower y crop}
      {0-9, 0-9},
      {0-9, 0-9},
      {0-9, 0-9},
      {0-9, 0-9},
      {0-9, 0-9}
    };

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0.0-9, 0-9, 0-9},
      {0-9, 0.0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0.0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9}
    };
  }
  
  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 0-9;
    public static final double FIELD_WIDTH_METERS = 0-9;

    public static final double RED_AMP_X = 14.613;
    public static final double RED_AMP_Y = 8.197;

    public static final double BLUE_AMP_X = 1.9;
    public static final double BLUE_AMP_Y = 8.161;
    public static final Rotation2d AMP_ROTATION = Rotation2d.fromDegrees(90);
    
    public static final double RED_SPEAKER_X = 16.511;
    public static final double RED_SPEAKER_Y = 5.529;

    public static final double BLUE_SPEAKER_X = 0;
    public static final double BLUE_SPEAKER_Y = 5.511;
  }

  public static final class JoystickConstants {
    public static final int DRIVER_LEFT_STICK_X = 0-9;
    public static final int DRIVER_LEFT_STICK_Y = 0-9;
    public static final int DRIVER_RIGHT_STICK_X = 0-9;
    public static final int DRIVER_RIGHT_BUMPER_ID = 0-9;
  }
  
  public static final class IntakeConstants {
    public static final int INTAKE_FRONT_MOTOR_ID = 0-9;
    public static final int INTAKE_BACK_MOTOR_ID = 0-9;
    public static final double INTAKE_FRONT_MOTOR_SPEED = 0.0;
    public static final double INTAKE_BACK_MOTOR_SPEED = 0.0;
  }

  public static final class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 0-9;
    public static final int RIGHT_MOTOR_ID = 0-9;
    public static final int PIVOT_MOTOR_ID = 0-9;

    public static final int SHOOTER_LIMIT_SWITCH_ID = 0-9;

    public static final int TURN_ENCODER_CHANNEL = 0;
    public static final double ANGLE_ZERO = 0.0;
    public static final SensorDirectionValue ENCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;

    public static final double LEFT_SHOOT_SPEAKER_RPM = 0-9;
    public static final double RIGHT_SHOOT_SPEAKER_RPM = 0-9;

    public static final double SHOOT_P = 0.0;
    public static final double SHOOT_I = 0.0;
    public static final double SHOOT_D = 0.0;

    public static final int SHOOT_AMP_ANGLE = 1-9;
    public static final int SHOOT_SPEAKER_ANGLE = 1-9;
    
    public static final double GEAR_RATIO_MOTOR_TO_FLYWHEEL = 30.0 / 40.0;

    public static final double AUTO_SHOOT_P = 0.0;
    public static final double AUTO_SHOOT_I = 0.0;
    public static final double AUTO_SHOOT_D = 0.0;
    public static final double AUTO_SHOOT_S = 0.0;
    public static final double AUTO_SHOOT_V = 0.0;
    public static Constraints AUTO_SHOOT_CONSTRAINTS = new Constraints(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 5);

    public static double[][] LEFT_MOTOR_SPEAKER_VALUES = {
      //{distance, rpm}
      {Units.feetToMeters(5.5), 1770},
      {Units.feetToMeters(7), 1665},
      {Units.feetToMeters(8.5), 1560}, // 1600  // 1615
      {Units.feetToMeters(10), 1350},
      {Units.feetToMeters(11.5), 1300},
      {Units.feetToMeters(13), 1280},
      {Units.feetToMeters(14.5), 1260},
      {Units.feetToMeters(16), 1300}
  };

    public static double[][] RIGHT_MOTOR_SPEAKER_VALUES = {
      //{distance, rpm}
      {Units.feetToMeters(5.5), 1770},
      {Units.feetToMeters(7), 1665},
      {Units.feetToMeters(8.5), 1560}, // 1600  // 1615
      {Units.feetToMeters(10), 1350},
      {Units.feetToMeters(11.5), 1300},
      {Units.feetToMeters(13), 1280},
      {Units.feetToMeters(14.5), 1260},
      {Units.feetToMeters(16), 1300}
    };

    public static double[][] SPEAKER_PIVOT_SPEAKER_POSITION = {
      //{distance, rpm}
      {Units.feetToMeters(5.5), 0},
      {Units.feetToMeters(7), 1},
      {Units.feetToMeters(8.5), 2}, // 1600  // 1615
      {Units.feetToMeters(10), 3},
      {Units.feetToMeters(11.5), 4},
      {Units.feetToMeters(13), 5},
      {Units.feetToMeters(14.5), 6},
      {Units.feetToMeters(16), 7}
    };

    public static double[][] LEFT_MOTOR_AMP_VALUES = {
      //{distance, rpm}
      {Units.feetToMeters(5.5), 1770},
      {Units.feetToMeters(7), 1665},
      {Units.feetToMeters(8.5), 1560}, // 1600  // 1615
      {Units.feetToMeters(10), 1350},
      {Units.feetToMeters(11.5), 1300},
      {Units.feetToMeters(13), 1280},
      {Units.feetToMeters(14.5), 1260},
      {Units.feetToMeters(16), 1300}
  };

    public static double[][] RIGHT_MOTOR_AMP_VALUES = {
      //{distance, rpm}
      {Units.feetToMeters(5.5), 1770},
      {Units.feetToMeters(7), 1665},
      {Units.feetToMeters(8.5), 1560}, // 1600  // 1615
      {Units.feetToMeters(10), 1350},
      {Units.feetToMeters(11.5), 1300},
      {Units.feetToMeters(13), 1280},
      {Units.feetToMeters(14.5), 1260},
      {Units.feetToMeters(16), 1300}
    };

    public static double[][] SPEAKER_PIVOT_AMP_POSITION = {
      //{distance, rpm}
      {Units.feetToMeters(5.5), 0},
      {Units.feetToMeters(7), 1},
      {Units.feetToMeters(8.5), 2}, // 1600  // 1615
      {Units.feetToMeters(10), 3},
      {Units.feetToMeters(11.5), 4},
      {Units.feetToMeters(13), 5},
      {Units.feetToMeters(14.5), 6},
      {Units.feetToMeters(16), 7}
    };
  }

  public static final class TrajectoryConstants {
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));
    public static final double MAX_SPEED = 5.7;
    public static final double MAX_ACCELERATION = 3.5;
    public static final double DEPLOYED_TRANSLATION_CONTROLLER_P = .35;
    public static final double DEPLOYED_THETA_CONTROLLER_P = .8;
    // public static final double REAL_TIME_X_CONTROLLER_P = 2;
    // public static final double REAL_TIME_Y_CONTROLLER_P = 2;
    // public static final double REAL_TIME_THETA_CONTROLLER_P = 3;
    // public static final double THETA_PROFILED_CONTROLLER_P = 1;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;
    // The length of the field in the x direction (left to right)
    public static final double FIELD_LENGTH_METERS = 16.54175;
    // The length of the field in the y direction (top to bottom)
    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(DEPLOYED_TRANSLATION_CONTROLLER_P);
    public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(DEPLOYED_THETA_CONTROLLER_P);
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;
  }
  
  public static final class TowerConstants {
    public static final int TOWER_MOTOR_ID = 0-9;
    public static final double TOWER_MOTOR_SPEED = 0.0;
    public static final int TOWER_BEAM_BREAK_BACK_PORT = 0-9;
    public static final int TOWER_BEAM_BREAK_FRONT_PORT = 0-9;
  }
}
