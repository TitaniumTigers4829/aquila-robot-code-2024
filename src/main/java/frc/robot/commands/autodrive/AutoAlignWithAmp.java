package frc.robot.commands.autodrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AutoAlignWithAmp extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;

  private final DoubleSupplier[] leftStick;

  private boolean isRed;
  private Pose2d ampPose;

  private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_LINEUP_ROTATION_P,
          ShooterConstants.AUTO_LINEUP_ROTATION_I,
          ShooterConstants.AUTO_LINEUP_ROTATION_D,
          ShooterConstants.AUTO_LINEUP_ROTATION_CONSTRAINTS);

  // private final ProfiledPIDController xTranslationController = new ProfiledPIDController(
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_P,
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_I,
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_D,
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_CONSTRAINTS
  // );

  // private final ProfiledPIDController yTranslationController = new ProfiledPIDController(
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_P,
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_I,
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_D,
  //   ShooterConstants.AUTO_LINEUP_TRANSLATION_CONSTRAINTS
  // );

  /** Creates a new AutoAlignWithAmp. */
  public AutoAlignWithAmp(
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem,
      DoubleSupplier[] leftStick) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.leftStick = leftStick;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // This will default to blue if it alliance isn't present
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    ampPose =
        isRed
            ? new Pose2d(
                FieldConstants.RED_AMP_SHOOT_X,
                FieldConstants.RED_AMP_SHOOT_Y,
                FieldConstants.RED_AMP_ROTATION)
            : new Pose2d(
                FieldConstants.BLUE_AMP_SHOOT_X,
                FieldConstants.BLUE_AMP_SHOOT_Y,
                FieldConstants.BLUE_AMP_ROTATION);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    super.execute();

    // Gets the error between the desired pos (the amp) and the current pos of the robot
    Pose2d drivePose = driveSubsystem.getPose();
    // double xPoseError = ampPose.getX() - drivePose.getX();
    // double yPoseError = ampPose.getY() - drivePose.getY();
    double thetaPoseError =
        ampPose.getRotation().getRadians() - drivePose.getRotation().getRadians();

    // Uses the PID controllers to calculate the drive output
    // double xOutput = deadband(xTranslationController.calculate(xPoseError, 0));
    // double yOutput = deadband(yTranslationController.calculate(yPoseError, 0));
    double turnOutput = deadband(turnController.calculate(thetaPoseError, 0));
    // SmartDashboard.putNumber("xOut", xOutput);
    // SmartDashboard.putNumber("yOut", yOutput);
    // SmartDashboard.putNumber("turnOut", turnOutput);

    // Gets the chassis speeds for the robot using the odometry rotation (not alliance relative)
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            leftStick[1].getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
            leftStick[0].getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
            turnOutput,
            driveSubsystem.getOdometryRotation2d());

    // Drives the robot towards the amp
    driveSubsystem.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false);

    // pivotSubsystem.setPivotAngle(PivotConstants.SHOOT_AMP_ANGLE);
    // shooterSubsystem.setRPM(ShooterConstants.SHOOT_AMP_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadband(double val) {
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  }
}
