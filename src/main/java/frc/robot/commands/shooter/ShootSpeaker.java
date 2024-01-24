// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.tower.TowerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootSpeaker extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem limelight;
  private final TowerSubsystem towerSubsystem;
  private final DriveSubsystem driveSubsystem;
  private double headingError = 0;
  private double[] shooterData;

  private DoubleSupplier leftX, leftY;
  ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P, ShooterConstants.AUTO_SHOOT_I, ShooterConstants.AUTO_SHOOT_D, null);
  
  /** Creates a new ShootSpeaker. */
  public ShootSpeaker(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, TowerSubsystem towerSubsystem, DriveSubsystem driveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelight = visionSubsystem;
    this.towerSubsystem = towerSubsystem;
    this.driveSubsystem = driveSubsystem;
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
    shooterData = limelight.getSpeakerShooterData();
    shooterSubsystem.setRPM(shooterData[0]);

    shooterSubsystem.setShooterPosition(shooterData[1]);

    if (isReadyToShoot()) {
      towerSubsystem.setTowerSpeed(TowerConstants.TOWER_MOTOR_SPEED);
    } else {
      towerSubsystem.setTowerSpeed(0);
    }

    headingError = shooterData[2];
    double output = turnController.calculate(headingError, 0);
    driveSubsystem.drive(leftX.getAsDouble(), leftY.getAsDouble(), output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setLeftMotorToNeutral();
    shooterSubsystem.setPivotMotorToNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
