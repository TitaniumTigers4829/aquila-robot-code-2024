// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private Spark led;
  private LEDProcess ledProcess;
  
  public LEDSubsystem() {
    this.led = new Spark(LEDConstants.LED_PORT);
    setProcess(LEDProcess.OFF);
  }

  public void setProcess(LEDProcess process) {
    ledProcess = process;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
