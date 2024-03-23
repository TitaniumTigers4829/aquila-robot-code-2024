// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ManualRollers extends Command {
  private ShooterSubsystem shooterSubsystem;
  private boolean direction;

  /** Creates a new ManualRollers. */
  public ManualRollers(ShooterSubsystem shooterSubsystem, boolean direction) {
    this.shooterSubsystem = shooterSubsystem;
    this.direction = direction;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setRollerSpeed(direction ? ShooterConstants.ROLLER_INTAKE_SPEED : -ShooterConstants.ROLLER_SHOOT_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
