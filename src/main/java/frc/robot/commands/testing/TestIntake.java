// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TestIntake extends Command {
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  /** Creates a new TestIntake. */
  public TestIntake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
    shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_NEUTRAL_SPEED);
    shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_NEUTRAL_SPEED);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
