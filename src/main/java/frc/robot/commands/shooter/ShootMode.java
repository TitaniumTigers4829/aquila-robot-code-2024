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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootMode extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;

  private final DoubleSupplier leftX, leftY;
  private final BooleanSupplier isFieldRelative;

  private double headingError = 0;
  private boolean movingNote = false;
  private int rollerCounter = 0;
  private boolean actualShoot;

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
  public ShootMode(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, BooleanSupplier isFieldRelative, Boolean actualShoot) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    this.isFieldRelative = isFieldRelative;
    this.actualShoot = actualShoot;
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
      isRed = true;
    }
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    
    // get positions of various things
    Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    // distance (for speaker lookups)
    double distance = robotPos.getDistance(speakerPos);
    // arctangent for desired heading
    desiredHeading = Math.atan2((speakerPos.getY() - robotPos.getY()), (speakerPos.getX() - robotPos.getX())) * 180.0 / Math.PI;
    // heading error (also used in isReadyToShoot())
    headingError = driveSubsystem.getHeading() - desiredHeading;
    // get PID output
    double turnOutput = turnController.calculate(headingError, 0);

    // allow the driver to drive slowly (NOT full speed - will mess up shooter)
    driveSubsystem.drive(
      leftX.getAsDouble(), 
      leftY.getAsDouble(), 
      turnOutput, 
      isFieldRelative.getAsBoolean()
    );

    shooterSubsystem.setRPM(6000);
    pivotSubsystem.setPivotFromDistance(distance);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      movingNote = true;
    }

    //Move note slightly backward so when you actually shoot it gives the note more time to get to 
    //prepare it to reach high velocity through swift shooter motors
    if (!actualShoot) {
      shooterSubsystem.setRollerSpeed(-0.1);
    }
    else if(actualShoot && movingNote) {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    }
    else if(!movingNote && actualShoot){
      SmartDashboard.putString("Should Never Reach for Shooting", "Not ready to shoot due to either pivot, shooter, or heading error.");
    }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot() {
    return Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_DEGREES && shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError();
  }

}
