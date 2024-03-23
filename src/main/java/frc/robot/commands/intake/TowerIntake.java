// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TowerIntake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem leds;
  private final boolean intakeReverse;
  
  public TowerIntake(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, boolean intakeReverse, LEDSubsystem leds) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeReverse = intakeReverse;
    this.leds = leds;
    addRequirements(intakeSubsystem);
  }
  
  /** Creates a new TowerIntake. */
  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);

    if (pivotSubsystem.isPivotWithinAcceptableError()) {
      if (intakeReverse) {
        leds.setProcess(LEDProcess.REVERSE_INTAKE);
        shooterSubsystem.setRollerSpeed(-ShooterConstants.ROLLER_INTAKE_SPEED); 
        intakeSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED);
        intakeSubsystem.setFlapperSpeed(-IntakeConstants.FLAPPER_SPEED);
      } else {
        if (shooterSubsystem.hasNote()) {
          intakeSubsystem.setIntakeSpeed(0);
          intakeSubsystem.setFlapperSpeed(0);
          shooterSubsystem.setRollerSpeed(0);
          leds.setProcess(LEDProcess.NOTE_IN);
          SmarterDashboardRegistry.noteIn();
        } else {
          leds.setProcess(LEDProcess.INTAKE);
          shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_INTAKE_SPEED);
          intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
          intakeSubsystem.setFlapperSpeed(IntakeConstants.FLAPPER_SPEED);
        }
      }
    }
  }
  

  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.DEFAULT);
    intakeSubsystem.setIntakeSpeed(0);
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.setPivotSpeed(0);
    intakeSubsystem.setFlapperSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}