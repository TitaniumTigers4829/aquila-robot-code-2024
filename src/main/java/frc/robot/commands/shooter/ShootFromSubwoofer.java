// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootFromSubwoofer extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  
  /** Creates a new ShootSpeaker. */
  public ShootFromSubwoofer(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setPivot(-0.15185546875 * 360.0);
    shooterSubsystem.setRPM(4000);

    if (pivotSubsystem.isPivotWithinAcceptableError() && shooterSubsystem.isShooterWithinAcceptableError()) {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE * 360.0);
    shooterSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
