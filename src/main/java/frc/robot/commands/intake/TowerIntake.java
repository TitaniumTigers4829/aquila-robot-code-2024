// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TowerIntake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem ledSubsystem;
  
  /** Creates a new TowerIntake. */
  public TowerIntake(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);

    shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);

    ledSubsystem.setProcess(LEDProcess.INTAKE);

    if(!shooterSubsystem.getSensor()) {
      ledSubsystem.setProcess(LEDProcess.NOTE_IN);
      intakeSubsystem.setIntakeSpeed(0);
      shooterSubsystem.setRollerSpeed(0);
    } else {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
    }
  }
  

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setProcess(LEDProcess.DEFAULT);
    intakeSubsystem.setIntakeSpeed(0);
    shooterSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}