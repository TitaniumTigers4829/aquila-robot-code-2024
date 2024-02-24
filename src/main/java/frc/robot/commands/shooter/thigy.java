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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class thigy extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;

  private double headingError = 0;
  private boolean movingNote = false;
  private int rollerCounter = 0;


  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation2d speakerPos;
  
  /** Creates a new ShootSpeaker. */
  public thigy( ShooterSubsystem shooterSubsystem,  PivotSubsystem pivotSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(shooterSubsystem, pivotSubsystem);
  }

  @Override
  // Called when the command is initially scheduled.
  public void initialize() {
    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // //if alliance is detected
    // if (alliance.isPresent()) {
    //   //and if it's red, we're red
    //   isRed = alliance.get() == Alliance.Red;
    // } else {
    //   //otherwise default to blue alliance
    //   isRed = true;
    // }
    // speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSubsystem.setRPM(4000);
    // pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE * 360.0);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot() {
    return  shooterSubsystem.isShooterWithinAcceptableError(); //&& pivotSubsystem.isPivotWithinAcceptableError(); //&&Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_DEGREES &&
  }

}
