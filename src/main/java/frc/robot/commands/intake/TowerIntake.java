// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final boolean intakeReverse;
  
  /** Creates a new TowerIntake. */
  public TowerIntake(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, boolean intakeReverse) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeReverse = intakeReverse;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);
    //intakeSubsystem.setIntakeAngle(IntakeConstants.OTB_INTAKE_ANGLE);

    if (pivotSubsystem.isPivotWithinAcceptableError()) { //intakeSubsystem.isIntakeWithinAcceptableError()
      if (intakeReverse) {
        shooterSubsystem.setRollerSpeed(-ShooterConstants.ROLLER_INTAKE_SPEED); //reverse otb?
        intakeSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED);
      } else {
        if (!shooterSubsystem.getSensor()) {
          intakeSubsystem.setIntakeSpeed(0);
          shooterSubsystem.setRollerSpeed(0);
        } else {
          shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_INTAKE_SPEED); //run otb
          intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
        }
      }
    }
  }
  

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}