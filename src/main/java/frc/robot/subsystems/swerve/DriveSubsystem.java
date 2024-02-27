package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.SmarterDashboardRegistry;


public class DriveSubsystem extends SubsystemBase {

  // This will stay the same throughout the match. These values are harder to test for and tune, so assume this guess is right.
  private static final Vector<N3> stateStandardDeviations = VecBuilder.fill(DriveConstants.X_POS_TRUST, DriveConstants.Y_POS_TRUST, Units.degreesToRadians(DriveConstants.ANGLE_TRUST));
  
  // This will be changed throughout the match depending on how confident we are that the limelight is right.
  private static final Vector<N3> visionMeasurementStandardDeviations = VecBuilder.fill(VisionConstants.VISION_X_POS_TRUST,
   VisionConstants.VISION_Y_POS_TRUST, Units.degreesToRadians(VisionConstants.VISION_ANGLE_TRUST));
  
  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule rearLeftSwerveModule;
  private final SwerveModule rearRightSwerveModule;

  private final AHRS gyro;
  private final SwerveDrivePoseEstimator odometry;
  private Command currentPathfindingCommand;

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
      DriveConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      "FL"
    );
    
    frontRightSwerveModule = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_CANCODER_ID,
      DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
      DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      "FR"
    );
    
    rearLeftSwerveModule = new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
      DriveConstants.REAR_LEFT_CANCODER_ID,
      DriveConstants.REAR_LEFT_ZERO_ANGLE,
      DriveConstants.REAR_LEFT_CANCODER_REVERSED,
      DriveConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
      DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
      "RL"
    );
    
    rearRightSwerveModule = new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
      DriveConstants.REAR_RIGHT_CANCODER_ID,
      DriveConstants.REAR_RIGHT_ZERO_ANGLE,
      DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
      DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
      "RR"
    );

    gyro = new AHRS(SPI.Port.kMXP);
  
    odometry = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match, use setPose() to set it in autonomous init
      stateStandardDeviations,
      visionMeasurementStandardDeviations
    );

    alliance = DriverStation.getAlliance();
    
    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      this::drive, 
      TrajectoryConstants.CONFIG,
      ()->false,
      this
    );
  }

  /**gets the chassis speeds */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
      frontLeftSwerveModule.getState(),
      frontRightSwerveModule.getState(),
      rearLeftSwerveModule.getState(),
      rearRightSwerveModule.getState()
      );
  }

  /**
   * Drives the robot using the joysticks.
   * @param xSpeed Speed of the robot in the x direction, positive being 
   * forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being
   * left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative
   * to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    // SmartDashboard.putBoolean("isFieldRelative", fieldRelative);
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d().plus(Rotation2d.fromDegrees(180)))
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
  }

  /**pid on the chassis rotation, used during auto */
  public void mergeDrive(ChassisSpeeds speeds, double rotationControl) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationControl, false);
  }

  /**
   * Returns the heading of the robot in degrees from 0 to 360. 
   * Counter-clockwise is positive. This factors in gyro offset.
   */
  public double getHeading() {
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  /**
   * Returns a Rotation2d for the heading of the robot.
   */  
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Returns a Rotation2d for the heading of the robot relative to the
   * field from the driver's perspective. This method is needed so that the
   * drive command and poseEstimator don't fight each other.
   */
  public Rotation2d getFieldRelativeRotation2d() {
    // Because the field isn't vertically symmetrical, we have the pose coordinates always start from the bottom left
    double rotationDegrees = getHeading();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      rotationDegrees += 180;
    } else {
      rotationDegrees += 0;
    }
    return Rotation2d.fromDegrees(rotationDegrees % 360);
  }
  
  /**
   * Sets the offset of the gyro.
   * @param gyroOffset The number of degrees that will be added to the
   * gyro's angle in getHeading.
   */
  public void setGyroOffset(double gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the estimated field-relative pose of the robot. Positive x 
   * being forward, positive y being left.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Updates the pose estimator with the pose calculated from the swerve
   * modules.
   */
  public void addPoseEstimatorSwerveMeasurement() {
    // TODO: experiment with using updateWithTime()
    odometry.update(
      getFieldRelativeRotation2d(),
      getModulePositions()
    );
  }

  /**
   * Updates the pose estimator with the pose calculated from the april
   * tags. How much it contributes to the pose estimation is set by
   * setPoseEstimatorVisionConfidence.
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the
   * pose from the april tags was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    odometry.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
    SmarterDashboardRegistry.setLimelightPose(visionMeasurement);
  }

  /**
   * Sets the standard deviations of model states, or how much the april
   * tags contribute to the pose estimation of the robot. Lower numbers
   * equal higher confidence and vice versa.
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

  /**
   * Resets the odometry to the specified pose, but keeps the current 
   * rotation.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Returns the current drivetrain position, as reported by the modules 
   * themselves. The order is: frontLeft, frontRight, backLeft, backRight
   * (should be the same as the kinematics).
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };

    return swerveModulePositions;
  }

  /**
   * Sets the modules to the specified states.
   * @param desiredStates The desired states for the swerve modules. The
   * order is: frontLeft, frontRight, backLeft, backRight (should be the 
   * same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  /**
   * builds a pathfinding command
   * @param finalX  final x pos of the path in meters
   * @param finalY final y pos of the path in meters
   * @param finalRot final rotation of the path in degrees
   * @return
   */
  public Command buildPathfindingCommand(double finalX, double finalY, double finalRot) {
    Pose2d endPose = new Pose2d(finalX, finalY, Rotation2d.fromDegrees(finalRot));

    // create the following command
    currentPathfindingCommand = new PathfindHolonomic(
      endPose,
      TrajectoryConstants.PATH_CONSTRAINTS,
      0.0, // end velocity
      this::getPose,
      this::getRobotRelativeSpeeds,
      this::drive,
      TrajectoryConstants.CONFIG,
      0.0, // distance to travel before rotating
      this
    );

    return currentPathfindingCommand;
  }

  /**
   * gets the pathfinding command
   */
  public Command getPathfindingCommand() {
    return currentPathfindingCommand;
  }

  /**
   * cancels the pathfinding command
   */
  public void cancelPathfindingCommand() {
    if (currentPathfindingCommand != null) {
      currentPathfindingCommand.cancel();
    }
  }

  public void periodic() {
    SmarterDashboardRegistry.setPose(getPose());
    SmartDashboard.putString("odometry", odometry.getEstimatedPosition().toString());
    SmartDashboard.putNumber("speakerDistance", getPose().getTranslation().getDistance(SmarterDashboardRegistry.getSpeakerPos()));
  }
}
