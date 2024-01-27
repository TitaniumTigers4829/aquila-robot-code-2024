package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.SmarterDashboardRegistry;


public class DriveSubsystem extends SubsystemBase {

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(DriveConstants.X_POS_TRUST, DriveConstants.Y_POS_TRUST, Units.degreesToRadians(DriveConstants.ANGLE_TRUST));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(VisionConstants.VISION_X_POS_TRUST,
   VisionConstants.VISION_Y_POS_TRUST, Units.degreesToRadians(VisionConstants.VISION_ANGLE_TRUST));
  

  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule rearLeftSwerveModule;
  private final SwerveModule rearRightSwerveModule;

  private final AHRS gyro;
  private final SwerveDrivePoseEstimator odometry;

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
      stateStdDevs,
      visionMeasurementStdDevs
    );

    alliance = DriverStation.getAlliance();
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    // SmartDashboard.putBoolean("isFieldRelative", fieldRelative);
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    //configurePath();
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
    gyro.reset();
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void addPoseEstimatorSwerveMeasurement() {
    odometry.update(
      getRotation2d(),
      getModulePositions()
    );
  }

    public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

    public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    odometry.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
    SmarterDashboardRegistry.setLimelightPose(visionMeasurement);
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
    Pose2d botPose = odometry.getEstimatedPosition();
    SmartDashboard.putString("odometry", botPose.toString());
    SmartDashboard.putNumberArray("botPose", new double[]{botPose.getX(), botPose.getY()});
    SmartDashboard.putNumber("pitch", getHeading());

    frontLeftSwerveModule.periodicFunction();
    frontRightSwerveModule.periodicFunction();
    rearLeftSwerveModule.periodicFunction();
    rearRightSwerveModule.periodicFunction();
  }

}