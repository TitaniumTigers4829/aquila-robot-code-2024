// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FlywheelSpinUpAuto extends Command {
  /** Creates a new FlywheelSpinUpAuto. */
  private final ShooterSubsystem shooterSubsystem;
  private final VisionSubsystem visionSubsystem;

  public FlywheelSpinUpAuto(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.hasNote() && visionSubsystem.canSeeAprilTags() && Units.metersToFeet(visionSubsystem.getDistanceFromClosestAprilTag()) <= 25) {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
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
}
