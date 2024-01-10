package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule rearLeftSwerveModule;
  private final SwerveModule rearRightSwerveModule;

  private final AHRS gyro;
  private final SwerveDriveOdometry odometry;

  private final Optional<DriverStation.Alliance> alliance;

  private double gyroOffset = 0.0;
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    frontLeftSwerveModule = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
      DriveConstants.FRONT_LEFT_CANCODER_ID,
      DriveConstants.FRONT_LEFT_ZERO_ANGLE,
      DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      "FL"
    );
    
    frontRightSwerveModule = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_CANCODER_ID,
      DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
      DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      "FR"
    );
    
    rearLeftSwerveModule = new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
      DriveConstants.REAR_LEFT_CANCODER_ID,
      DriveConstants.REAR_LEFT_ZERO_ANGLE,
      DriveConstants.REAR_LEFT_CANCODER_REVERSED,
      DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
      "RL"
    );
    
    rearRightSwerveModule = new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
      DriveConstants.REAR_RIGHT_CANCODER_ID,
      DriveConstants.REAR_RIGHT_ZERO_ANGLE,
      DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
      DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
      "RR"
    );

    gyro = new AHRS(SPI.Port.kMXP);
  
    odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d() // This is the position for where the robot starts the match
    );

    alliance = DriverStation.getAlliance();
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    // SmartDashboard.putBoolean("isFieldRelative", fieldRelative);
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getFieldRelativeRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    // configurePath();
  }
  public double getHeading() {
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Rotation2d getFieldRelativeRotation2d() {
    // Because the field isn't vertically symmetrical, we have the pose coordinates always start from the bottom left
    double rotationDegrees = getHeading();
    if (alliance.isPresent() && alliance.get()==DriverStation.Alliance.Blue) {
      rotationDegrees += 0;
    } else {
      rotationDegrees += 180;
    }
    return Rotation2d.fromDegrees(rotationDegrees % 360);
  }
                                                                       
  public void setGyroOffset(double gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  public void zeroHeading() {
    double rotationDegrees = getHeading();
    if (alliance.isPresent() && alliance.get()==DriverStation.Alliance.Blue) {
      rotationDegrees += 0;
    } else {
      rotationDegrees += 180;
    }
    gyroOffset = (rotationDegrees % 360);
    gyro.reset();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }
 
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };

    return swerveModulePositions;
  }

  public void periodic() {
    // Pose2d estimatedPose = odometry.getEstimatedPosition();
    // SmartDashboardLogger.infoString("Estimated pose", estimatedPose.toString());
    
    // //smarterdashboard:
    // SmarterDashboardRegistry.setPose(estimatedPose);
    //                                         //  pitch, roll, yaw
    // SmarterDashboardRegistry.setOrientation(getHeading(), 0, 0);

    frontLeftSwerveModule.periodicFunction();
    frontRightSwerveModule.periodicFunction();
    rearLeftSwerveModule.periodicFunction();
    rearRightSwerveModule.periodicFunction();
  }
  // commented out pathplanner beta code:
  // public ChassisSpeeds getRobotRelativeSpeeds(){
  //   return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  // }

  // public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
  //     SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(robotRelativeSpeeds);

  //     SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

  //     setModuleStates(states);
  // }

  // public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
  //     ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation());

  //     driveRobotRelative(robotRelative);
  // }

  // public SwerveModuleState[] getModuleStates(){
  //   SwerveModuleState[] swerveModuleStates = {
  //       frontLeftSwerveModule.getState(),
  //       frontRightSwerveModule.getState(),
  //       rearLeftSwerveModule.getState(),
  //       rearRightSwerveModule.getState()
  //     };
  //   return swerveModuleStates;
  // }

  // private void configurePath() {
  //   // Configure the AutoBuilder last
  //   AutoBuilder.configureHolonomic(
  //     this::getPose, // Robot pose supplier
  //     this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
  //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //     this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  //         new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
  //         4.5, // Max module speed, in m/s
  //         0.4, // Drive base radius in meters. Distance from robot center to furthest module.
  //         new ReplanningConfig() // Default path replanning config. See the API for the options here
  //     ),
  //     this // Reference to this subsystem to set requirements
  //   );
  // };
}