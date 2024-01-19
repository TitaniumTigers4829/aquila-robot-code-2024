// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

public class TowerSubsystem extends SubsystemBase {
  private final TalonFX towerMotor;
  DigitalInput beamBreakBack;
  DigitalInput beamBreakFront;

  /** Creates a new TowerSubsystem. */
  public TowerSubsystem() {
    towerMotor = new TalonFX(TowerConstants.TOWER_MOTOR_ID);
    beamBreakBack = new DigitalInput(TowerConstants.TOWER_BEAM_BREAK_BACK_PORT);
    beamBreakFront = new DigitalInput(TowerConstants.TOWER_BEAM_BREAK_FRONT_PORT);
  }
  
  public void setTowerSpeed(double speed) {
    towerMotor.set(speed);
  }

  public boolean hasNoteBottom() {
    return beamBreakBack.get();
  }

  public boolean hasNoteTop() {
    return beamBreakFront.get();
  }

  @Override
  public void periodic() {
  }
}