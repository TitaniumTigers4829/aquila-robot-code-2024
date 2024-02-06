// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootSpeaker extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;

  private final DoubleSupplier leftX, leftY;
  private final BooleanSupplier isFieldRelative;

  private double headingError = 0;

  private final ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
  );

  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation2d speakerPos;
  
  /** Creates a new ShootSpeaker. */
  public ShootSpeaker(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, BooleanSupplier isFieldRelative) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    this.isFieldRelative = isFieldRelative;
    addRequirements(shooterSubsystem, driveSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    //if alliance is detected
    if (alliance.isPresent()) {
      //and if it's red, we're red
      isRed = alliance.get() == Alliance.Red;
    } else {
      //otherwise default to blue alliance
      isRed = false;
    }
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
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
    headingError = driveSubsystem.getHeading() - desiredHeading;
    // get PID output
    double turnOutput = turnController.calculate(headingError, 0);

    double shootDistance = shooterSubsystem.getShooterTargetRPM(distance);
    double pivotTarget = pivotSubsystem.getPivotTarget(distance);
    // allow the driver to drive slowly (NOT full speed - will mess up shooter)
    driveSubsystem.drive(
      leftX.getAsDouble(), 
      leftY.getAsDouble(), 
      turnOutput, 
      isFieldRelative.getAsBoolean()
    );

    shooterSubsystem.setShooterRPMFromDistance(distance);
    pivotSubsystem.setPivotFromDistance(distance);

    
    // if we are ready to shoot:
    if (isReadyToShoot(shootDistance, pivotTarget)) {
      // feed the note into the flywheels
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    pivotSubsystem.setPivotMotorToNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot(double shootTargetDistance, double pivotTargetPos) {
    return Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_DEGREES && shooterSubsystem.isShooterWithinAcceptableError(shootTargetDistance) && pivotSubsystem.isPivotWithinAcceptableError(pivotTargetPos);
  }

}
