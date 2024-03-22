// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class SetPivotAngle extends Command {
  PivotSubsystem pivotSubsystem;
  double angle;

  /** Creates a new SetPivotAngle. */
  public SetPivotAngle(PivotSubsystem pivotSubsystem, double angle) {
    this.pivotSubsystem = pivotSubsystem;
    this.angle = angle;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
