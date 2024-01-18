// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class SetMotorPos extends Command {
  DriveSubsystem driveSubsystem;
  DoubleSupplier pos;

  /** Creates a new SetMotorPos. */
  public SetMotorPos(DriveSubsystem driveSubsystem, DoubleSupplier pos) {
    this.driveSubsystem = driveSubsystem;
    this.pos = pos;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setMotorPos(pos.getAsDouble() / 2.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
