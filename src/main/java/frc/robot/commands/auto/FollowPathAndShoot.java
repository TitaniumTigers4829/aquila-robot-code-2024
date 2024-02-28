// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowPathAndShoot extends DriveCommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private PivotSubsystem pivotSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private Command controllerCommand;
  private Translation2d speakerPos;
  private boolean isRed;
  private double rotationControl;
  private double desiredHeading;
  private double headingError = 0;
  private double headingOffset = 0;

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
  );

  /** Creates a new FollowPathAndShoot. */
  public FollowPathAndShoot(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, String path) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    controllerCommand = Choreo.choreoSwerveCommand(
      Choreo.getTrajectory(path),
      driveSubsystem::getPose, 
      new PIDController(TrajectoryConstants.REALTIME_TRANSLATION_CONTROLLER_P, 0, 0), 
      new PIDController(TrajectoryConstants.REALTIME_TRANSLATION_CONTROLLER_P, 0, 0), 
      new PIDController(TrajectoryConstants.REALTIME_THETA_CONTROLLER_P, 0, 0), 
      (ChassisSpeeds speeds) -> driveSubsystem.mergeDrive(speeds, rotationControl),
        ()->false,
      // TODO: scuffed
      driveSubsystem);
    addRequirements(visionSubsystem, pivotSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerCommand.schedule();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRed = alliance.get() == Alliance.Red;
    } else {
      isRed = true;
    }
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    headingOffset = isRed ? -1 * TrajectoryConstants.AUTO_SHOOT_HEADING_OFFSET : TrajectoryConstants.AUTO_SHOOT_HEADING_OFFSET;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    // get positions of various things
    Translation2d robotPos = SmarterDashboardRegistry.getPose().getTranslation();
    // distance (for speaker lookups)
    double distance = robotPos.getDistance(speakerPos);
    // arctangent for desired heading
    desiredHeading = Math.atan2((speakerPos.getY() - robotPos.getY()), (speakerPos.getX() - robotPos.getX())) * 180.0 / Math.PI;
    // heading error (also used in isReadyToShoot())
    headingError = driveSubsystem.getHeading() - desiredHeading - headingOffset;
    // get PID output
    rotationControl = thetaController.calculate(headingError, 0);


    // if we are ready to shoot:
    if (shooterSubsystem.isReadyToShoot(headingError) && pivotSubsystem.isPivotWithinAcceptableError()) {
    shooterSubsystem.setRPM(6000);
    pivotSubsystem.setPivotFromDistance(distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }

}
