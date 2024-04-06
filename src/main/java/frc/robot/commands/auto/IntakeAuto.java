// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

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

public class IntakeAuto extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem leds;
  private boolean intakeSensorLatch;
  
  public IntakeAuto(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem leds) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.leds = leds;
    addRequirements(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds);
  }

  @Override
  public void initialize() {
    intakeSensorLatch = false;
    // the reason for doing the ledprocess for the intake here is a little complicated:
    // when a note passes by the sensor, it will briefly be tripped, causing intakeSubsystem.sensorDetectsNote()
    // to briefly return true. By doing it this way, the LEDs will be red (LEDProcess.INTAKE) until the
    // intake sensor detects the note, causing the LEDs to turn yellow (LEDProcess.NOTE_HALFWAY_IN)
    // If it were done the way it was before this, they would briefly flash yellow before going back to red
    leds.setProcess(LEDProcess.INTAKE);
  }
  
  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);

  if (pivotSubsystem.isPivotWithinAcceptableError()) {
      if (shooterSubsystem.hasNote()) {
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setFlapperSpeed(0);
        shooterSubsystem.setRollerSpeed(0);
        leds.setProcess(LEDProcess.NOTE_IN);
        SmarterDashboardRegistry.noteIn();
      } else if (intakeSubsystem.sensorDetectsNote()) {
        intakeSensorLatch = true;
        leds.setProcess(LEDProcess.NOTE_HALFWAY_IN);
      } else {
        if (!intakeSensorLatch) {
          shooterSubsystem.setRollerSpeed(0.3);
          intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
        } else {
          shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_INTAKE_BEFORE_LATCH_SPEED);
          intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
        }
        intakeSubsystem.setFlapperSpeed(IntakeConstants.FLAPPER_SPEED);
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
    return shooterSubsystem.hasNote();
  }
}