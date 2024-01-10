// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(0-9);

    BaseStatusSignal.setUpdateFrequencyForAll(250, 
    intakeMotor.getPosition(), 
    intakeMotor.getVelocity(), 
    intakeMotor.getAcceleration(), 
    intakeMotor.getMotorVoltage());

    intakeMotor.optimizeBusUtilization(30);
  }

  public void intake(double speed) {
    // TODO: do somethin
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
