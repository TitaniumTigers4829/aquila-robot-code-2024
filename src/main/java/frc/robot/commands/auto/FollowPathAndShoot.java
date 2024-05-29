// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

public class FollowPathAndShoot extends DriveCommandBase {
  private DriveSubsystem driveSubsystem;
  private PivotSubsystem pivotSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private Command controllerCommand;
  private Translation3d speakerPos;
  private boolean isRed;
  private double rotationControl;
  private double desiredHeading;
  private double headingError = 0;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_SHOOT_P,
          ShooterConstants.AUTO_SHOOT_I,
          ShooterConstants.AUTO_SHOOT_D,
          ShooterConstants.AUTO_SHOOT_CONSTRAINTS);

  /** Creates a new FollowPathAndShoot. */
  public FollowPathAndShoot(
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem,
      String path,
      boolean resetOdometry) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    ChoreoTrajectory traj = Choreo.getTrajectory(path);
    if (resetOdometry) {
      driveSubsystem.resetOdometry(traj.getInitialPose());
    }
    controllerCommand =
        Choreo.choreoSwerveCommand(
            traj,
            driveSubsystem::getPose,
            new PIDController(TrajectoryConstants.AUTO_TRANSLATION_P, 0, 0),
            new PIDController(TrajectoryConstants.AUTO_TRANSLATION_P, 0, 0),
            new PIDController(TrajectoryConstants.AUTO_THETA_P, 0, 0),
            (ChassisSpeeds speeds) -> mergeDrive(speeds),
            () -> false,
            driveSubsystem);
    addRequirements(visionSubsystem, pivotSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerCommand.initialize();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRed = alliance.get() == Alliance.Red;
    } else {
      isRed = true;
    }
    speakerPos =
        isRed
            ? new Translation3d(
                FieldConstants.RED_SPEAKER_X,
                FieldConstants.RED_SPEAKER_Y,
                ShooterConstants.SPEAKER_HEIGHT)
            : new Translation3d(
                FieldConstants.BLUE_SPEAKER_X,
                FieldConstants.BLUE_SPEAKER_Y,
                ShooterConstants.SPEAKER_HEIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    controllerCommand.execute();

    Translation2d robotPose2d = driveSubsystem.getPose().getTranslation();
    Translation3d robotPos3d =
        new Translation3d(robotPose2d.getX(), robotPose2d.getY(), ShooterConstants.SHOOTER_HEIGHT);
    // speeds
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    // this gets the time that the note will be in the air between the robot and the speaker
    double tmpDist = robotPos3d.getDistance(speakerPos);
    double dt = tmpDist / ShooterConstants.NOTE_LAUNCH_VELOCITY;
    // For shooting while moving, we can pretend that our robot is stationary, but has traveled
    // the distance that was how long the note was in the air for times the robots current velocity
    double dx = speeds.vxMetersPerSecond * dt * ((tmpDist * 0.05) + 1);
    double dy = speeds.vyMetersPerSecond * dt * ((tmpDist * 0.05) + 1);
    // account for the current velocity:
    robotPose2d.plus(new Translation2d(dx, dy).rotateBy(driveSubsystem.getOdometryRotation2d()));
    // continue the command as normal
    double distance = robotPose2d.getDistance(speakerPos.toTranslation2d());
    desiredHeading =
        Math.atan2(robotPose2d.getY() - speakerPos.getY(), robotPose2d.getX() - speakerPos.getX());
    // heading error
    headingError = desiredHeading - driveSubsystem.getOdometryRotation2d().getRadians();
    // get PID output
    rotationControl = thetaController.calculate(headingError, 0);

    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    pivotSubsystem.setPivotFromDistance(distance);

    // if we are ready to shoot:
    if (shooterSubsystem.isReadyToShoot(headingError)
        && pivotSubsystem.isPivotWithinAcceptableError()
        && Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_MOVING_RADIANS) {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setRollerSpeed(0);
    shooterSubsystem.setSpeed(0);
    pivotSubsystem.setPivotSpeed(0);
    controllerCommand.end(interrupted);
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }

  private void mergeDrive(ChassisSpeeds speeds) {
    driveSubsystem.drive(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationControl, false);
  }
}
