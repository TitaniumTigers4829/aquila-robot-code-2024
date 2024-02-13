// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TowerIntake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  
  /** Creates a new TowerIntake. */
  public TowerIntake(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);

    shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
    // if(!shooterSubsystem.getSensor()) {
    //   intakeSubsystem.setIntakeSpeed(0);
    //   shooterSubsystem.setRollerSpeed(0);
    // } else {
    //   shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    //   intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
    // }
  }
  

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    // shooterSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}