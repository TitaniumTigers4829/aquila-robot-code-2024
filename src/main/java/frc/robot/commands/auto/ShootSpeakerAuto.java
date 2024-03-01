// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootSpeakerAuto extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final VisionSubsystem visionSubsystem;

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
  public ShootSpeakerAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.visionSubsystem = visionSubsystem;
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
      //otherwise default to red alliance
      isRed = true;
    }

    SmartDashboard.putBoolean("red", isRed);
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    SmartDashboard.putString("speakerPos", speakerPos.toString());
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
    if (isRed) {
      desiredHeading = Math.atan2((speakerPos.getY() - robotPos.getY()), (speakerPos.getX() - robotPos.getX()));
    } else {
      desiredHeading = Math.atan2((robotPos.getY() - speakerPos.getY()), (robotPos.getX() - speakerPos.getX()));
    }
    // heading error (also used in isReadyToShoot())
    headingError = desiredHeading - driveSubsystem.getRotation2d().getRadians();
    // get PID output
    // SmartDashboard.putNumber("desired Heading", desiredHeading);
    // SmartDashboard.putNumber("drivetrain error", headingError);
    // SmartDashboard.putNumber("current heading", driveSubsystem.getRotation2d().getRadians());
    // SmartDashboard.putNumber("turnOutput", turnOutput);
    SmartDashboard.putNumber("speakerDistance", distance);


    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    pivotSubsystem.setPivotFromDistance(distance);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot() {
    return shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError();
  }

  private double deadband(double val) {
    if (Math.abs(val) < 0.1) {
      return 0.0;
    } else {
      return val;
    }
  } 
}
