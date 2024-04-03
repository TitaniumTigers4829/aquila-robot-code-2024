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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SpinUpForSpeakerAuto extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem leds;


  /** Creates a new SpinUpForSpeaker. */
  public SpinUpForSpeakerAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, BooleanSupplier isFieldRelative, LEDSubsystem leds) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leds = leds;
    addRequirements(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    
    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM * 0.75);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      leds.setProcess(LEDProcess.SHOOT);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.DEFAULT);
    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM * 0.75);

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
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  } 
}
