// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.tower.TowerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootAmp extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem limelight;
  private final TowerSubsystem towerSubsystem;

  private DoubleSupplier leftX, leftY;

  private double desiredHeading;
  private double headingError = 0;
  private double[] stuff;

  ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
  );
  
  /** Creates a new ShootSpeaker. */
  public ShootAmp(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, TowerSubsystem towerSubsystem, DoubleSupplier leftX, DoubleSupplier leftY) {
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = visionSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    addRequirements(shooterSubsystem, visionSubsystem, towerSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private boolean isReadyToShoot() {
    return (Math.abs(headingError) < 3) && (shooterSubsystem.isShooterWithinAcceptableError()) && (shooterSubsystem.isPivotWithinAcceptableError());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stuff = limelight.getAmpStuff();
    shooterSubsystem.setRPM(stuff[0], stuff[1]);

    shooterSubsystem.setAmpPosition(stuff[2]);

    if (isReadyToShoot()) {
      towerSubsystem.setTowerSpeed(TowerConstants.TOWER_MOTOR_SPEED);
    } else {
      towerSubsystem.setTowerSpeed(0);
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed;
    if (alliance.isPresent()) {
      isRed = alliance.get() == Alliance.Red;
    } else {
      isRed = true;
    }
    Translation2d targetLocation = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    targetLocation.minus(SmarterDashboardRegistry.getPose().getTranslation());
    desiredHeading = Math.atan(targetLocation.getY() / targetLocation.getX());
    headingError = SmarterDashboardRegistry.getPose().getRotation().getRadians() - desiredHeading;

    double output = turnController.calculate(headingError, 0);
    driveSubsystem.drive(leftX.getAsDouble(), leftY.getAsDouble(), output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setLeftMotorToNeutral();
    shooterSubsystem.setRightMotorToNeutral();
    shooterSubsystem.setPivotMotorToNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
