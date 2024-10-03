package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.extras.swerve.ModuleLimits;
import frc.robot.extras.swerve.SwerveSetpoint;
import frc.robot.extras.swerve.SwerveSetpointGenerator;
import java.util.Optional;

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

  private boolean collisionDetected = false;
  private boolean isSlipping = false;

  private double lastWorldLinearAccelX;
  private double lastWorldLinearAccelY;
  private double currentJerkX;
  private double currentJerkY;
  private static final ModuleLimits currentModuleLimits =
      new ModuleLimits(
          DriveConstants.MAX_SPEED_METERS_PER_SECOND,
          4,
          DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint currentSetpoint;

  private Optional<DriverStation.Alliance> alliance;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeftSwerveModule =
        new SwerveModule(
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
            DriveConstants.FRONT_LEFT_CANCODER_ID,
            DriveConstants.FRONT_LEFT_ZERO_ANGLE,
            DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
            "FL");

    frontRightSwerveModule =
        new SwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            DriveConstants.FRONT_RIGHT_CANCODER_ID,
            DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
            DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
            "FR");

    rearLeftSwerveModule =
        new SwerveModule(
            DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
            DriveConstants.REAR_LEFT_CANCODER_ID,
            DriveConstants.REAR_LEFT_ZERO_ANGLE,
            DriveConstants.REAR_LEFT_CANCODER_REVERSED,
            DriveConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
            "RL");

    rearRightSwerveModule =
        new SwerveModule(
            DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
            DriveConstants.REAR_RIGHT_CANCODER_ID,
            DriveConstants.REAR_RIGHT_ZERO_ANGLE,
            DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
            DriveConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
            "RR");

    gyro = new AHRS(SPI.Port.kMXP);

    odometry =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            getGyroRotation2d(),
            getModulePositions(),
            new Pose2d(), // This is the position for where the robot starts the match, use
            // resetOdometry() to set it in autonomous init
            stateStandardDeviations,
            visionMeasurementStandardDeviations);

    alliance = DriverStation.getAlliance();

    setpointGenerator = new SwerveSetpointGenerator(DriveConstants.DRIVE_KINEMATICS, DriveConstants.MODULE_TRANSLATIONS);
    currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());

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

  /** gets the chassis speeds */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
        frontLeftSwerveModule.getState(),
        frontRightSwerveModule.getState(),
        rearLeftSwerveModule.getState(),
        rearRightSwerveModule.getState());
  }

  /**
   * Drives the robot using the joysticks.
   *
   * @param xSpeed Speed of the robot in the x direction, positive being forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    desiredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotationSpeed, getOdometryAllianceRelativeRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);

    currentSetpoint =
        setpointGenerator.generateSetpoint(
            currentModuleLimits, currentSetpoint, desiredSpeeds, 0.02);

    setModuleStates(currentSetpoint.moduleStates());
  }

  public void drive(ChassisSpeeds speeds) {
    drive(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
  }

  /**
   * Allows PID on the chassis rotation.
   *
   * @param speeds The ChassisSpeeds of the drive to set.
   * @param rotationControl The control on the drive rotation.
   */
  public void mergeDrive(ChassisSpeeds speeds, double rotationControl) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationControl, false);
  }

   /**
     * the method comes from 1690's <a href="https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674">online software session</a>
     * gets the skidding ratio from the latest , that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the  ratio between the maximum and minimum magnitude of the "translational" part of the speed of the modules
     *
     * @param swerveStatesMeasured the swerve states measured from the modules
     * @param swerveDriveKinematics the kinematics
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     * */
    public static double getSkiddingRatio(SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
        final double angularVelocityOmegaMeasured = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
        final double[] swerveStatesTranslationalPartMagnitudes = new double[swerveStatesMeasured.length];

        for (int i =0; i < swerveStatesMeasured.length; i++) {
            final Translation2d swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
                    swerveStatesRotationalPartAsVector = convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
                    swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
        for (double translationalSpeed:swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

    private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
        return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }

  public double getSkidRatio() {
    return getSkiddingRatio(moduleStates(), DriveConstants.DRIVE_KINEMATICS);
  }

  public boolean isSkidding() {

    double skidRatio = getSkidRatio();
    boolean collisionDetected = isCollisionDetected();
    boolean isStatorGood = getStatorCurrents();
    // boolean
    if (skidRatio > 1 || collisionDetected || !isStatorGood) {
      return true;
    }
    return false;
  }

  public SwerveModuleState[] moduleStates() {
    SwerveModuleState[] states = {
      frontLeftSwerveModule.getState(),
      frontRightSwerveModule.getState(),
      rearLeftSwerveModule.getState(),
      rearRightSwerveModule.getState()
    };
    return states;
  }


  /** Runs in a circle at omega. */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    drive(0, 0, omegaSpeed, false);
  }

  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    double[] wheelPositions = {
      frontLeftSwerveModule.getDrivePositionRadians(),
      frontRightSwerveModule.getDrivePositionRadians(),
      rearLeftSwerveModule.getDrivePositionRadians(),
      rearRightSwerveModule.getDrivePositionRadians()
    };
    return wheelPositions;
  }

  /**
   * Returns the heading of the robot in degrees from 0 to 360.
   *
   * @return Value is Counter-clockwise positive.
   */
  public double getHeading() {
    return -gyro.getAngle();
  }

  /**
   * Gets the rate of rotation of the NavX.
   *
   * @return The current rate in degrees per second.
   */
  public double getGyroRate() {
    return -gyro.getRate();
  }

  public boolean isCollisionDetected() {

    double currWorldLinearAccelX = gyro.getWorldLinearAccelX();
    currentJerkX = currWorldLinearAccelX - lastWorldLinearAccelX;
    lastWorldLinearAccelX = currWorldLinearAccelX;
    double currWorldLinearAccelY = gyro.getWorldLinearAccelY();
    currentJerkY = currWorldLinearAccelY - lastWorldLinearAccelY;
    lastWorldLinearAccelY = currWorldLinearAccelY;

    if ( ( Math.abs(currentJerkX) > DriveConstants.COLLISION_THRESHOLD_DELTA_G ) ||
         ( Math.abs(currentJerkY) >  DriveConstants.COLLISION_THRESHOLD_DELTA_G ) ) {
       return collisionDetected = true;
    }
    return collisionDetected = false;
  }

  /**
   * Returns a Rotation2d for the heading of the robot.
   */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /** Returns a Rotation2d for the heading of the robot. */
  public Rotation2d getGyroFieldRelativeRotation2d() {
    return Rotation2d.fromDegrees(getHeading() + getAllianceAngleOffset());
  }

  /** Returns 0 degrees if the robot is on the blue alliance, 180 if on the red alliance. */
  public double getAllianceAngleOffset() {
    alliance = DriverStation.getAlliance();
    double offset =
        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180.0 : 0.0;
    return offset;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the estimated field-relative pose of the robot. Positive x being forward, positive y
   * being left.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /** Returns a Rotation2d for the heading of the robot */
  public Rotation2d getOdometryRotation2d() {
    return getPose().getRotation();
  }

  /**
   * Returns a Rotation2d for the heading of the robot relative to the field from the driver's
   * perspective. This method is needed so that the drive command and poseEstimator don't fight each
   * other. It uses odometry rotation.
   */
  public Rotation2d getOdometryAllianceRelativeRotation2d() {
    return getPose().getRotation().plus(Rotation2d.fromDegrees(getAllianceAngleOffset()));
  }

  /** Updates the pose estimator with the pose calculated from the swerve modules. */
  public void addPoseEstimatorSwerveMeasurement() {
    odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(), getModulePositions());
  }

  /**
   * Updates the pose estimator with the pose calculated from the april tags. How much it
   * contributes to the pose estimation is set by setPoseEstimatorVisionConfidence.
   *
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the pose from the april tags
   *     was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(
      Pose2d visionMeasurement, double currentTimeStampSeconds) {
    odometry.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  /**
   * Sets the standard deviations of model states, or how much the april tags contribute to the pose
   * estimation of the robot. Lower numbers equal higher confidence and vice versa.
   *
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(
      double xStandardDeviation, double yStandardDeviation, double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(
        VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

  /** Resets the odometry to the specified pose and rotation. */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  /**
   * Gets the current module positions.
   *
   * @return The current drivetrain position, as reported by the modules themselves. The order is:
   *     frontLeft, frontRight, backLeft, backRight (should be the same as the kinematics).
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
   *
   * @param desiredStates The desired states for the swerve modules. The order is: frontLeft,
   *     frontRight, backLeft, backRight (should be the same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  public boolean[] getStatorStatus() {
    boolean[] stators = {
      frontLeftSwerveModule.isDriveStatorGood() || frontLeftSwerveModule.isTurnStatorGood(),
      frontRightSwerveModule.isDriveStatorGood() || frontRightSwerveModule.isTurnStatorGood(),
      rearLeftSwerveModule.isDriveStatorGood() || frontLeftSwerveModule.isTurnStatorGood(),
      rearRightSwerveModule.isDriveStatorGood() || rearRightSwerveModule.isTurnStatorGood()
    };
    return stators;
  }

  public boolean getStatorCurrents() {
    boolean[] statorStatus = getStatorStatus();
    for (int i = 0; i < statorStatus.length; i++) {
        if (statorStatus[i]) {
          return true;
        }
    }
    return false;
}

public boolean isSlipping() {
  if (getSkidRatio() > 1.0) {
    return true;
  }
  return false;
}

  /**
   * Gets the swerve module states
   *
   * @return the module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      frontLeftSwerveModule.getState(),
      frontRightSwerveModule.getState(),
      rearLeftSwerveModule.getState(),
      rearRightSwerveModule.getState()
    };
    return states;
  }

  public double[] getModuleAngles() {
    double[] angles = {
      frontLeftSwerveModule.getState().angle.getRadians(),
      frontRightSwerveModule.getState().angle.getRadians(),
      rearLeftSwerveModule.getState().angle.getRadians(),
      rearRightSwerveModule.getState().angle.getRadians()
    };
    return angles;
  }

  public void periodic() {
    Pose2d pose = getPose();
    SmartDashboard.putBoolean("screwed", Math.abs(pose.getX()) > 20);
    SmartDashboard.putString("odometry", pose.toString());
    SmartDashboard.putNumber("speakerDistance", pose.getTranslation().getDistance(SmarterDashboardRegistry.getSpeakerPos()));
  }
}
