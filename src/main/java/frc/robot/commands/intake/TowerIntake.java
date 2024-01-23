// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.tower.TowerSubsystem;

public class TowerIntake extends Command {
  private final TowerSubsystem tower;
  private final IntakeSubsystem intake;
  
  /** Creates a new TowerIntake. */
  public TowerIntake(TowerSubsystem towerSubsystem, IntakeSubsystem intakeSubsystem) {
    this.tower = towerSubsystem;
    this.intake = intakeSubsystem;
    addRequirements(towerSubsystem, intakeSubsystem);
  }

  @Override
  public void execute() {
    if (tower.hasNoteBottom()) {
      tower.setTowerSpeed(TowerConstants.TOWER_MOTOR_SPEED);
      intake.setIntakeFrontSpeed(0);
    }
    if (tower.hasNoteTop()) {
      tower.setTowerSpeed(0);
      intake.setIntakeFrontSpeed(0);
    }
    else {
      tower.setTowerSpeed(TowerConstants.TOWER_MOTOR_SPEED);
      intake.setIntakeFrontSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
