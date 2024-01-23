// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

public class TowerSubsystem extends SubsystemBase {
  private final TalonFX towerMotor;
  private final DigitalInput bottomBeamBreak;
  private final DigitalInput topBeamBreak;

  /** Creates a new TowerSubsystem. */
  public TowerSubsystem() {
    towerMotor = new TalonFX(TowerConstants.TOWER_MOTOR_ID);
    bottomBeamBreak = new DigitalInput(TowerConstants.TOWER_BOTTOM_BEAMBREAK_PORT);
    topBeamBreak = new DigitalInput(TowerConstants.TOWER_TOP_BEAMBREAK_PORT);
  }
  
  public void setTowerSpeed(double speed) {
    towerMotor.set(speed);
  }

  public boolean hasNoteBottom() {
    return bottomBeamBreak.get();
  }

  public boolean hasNoteTop() {
    return topBeamBreak.get();
  }

  @Override
  public void periodic() {
  }
}