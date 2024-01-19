// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeFrontMotor;
  private final TalonFX intakeBackMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeFrontMotor = new TalonFX(IntakeConstants.INTAKE_FRONT_MOTOR_ID);
    intakeBackMotor = new TalonFX(IntakeConstants.INTAKE_BACK_MOTOR_ID);
  }

  public void setIntakeFrontSpeed(double speed) {
    // TODO: set front intake motor speed
    intakeFrontMotor.set(speed);
  }

  public void setIntakeBackSpeed(double speed) {
    // TODO: set back intake motor speed
    intakeBackMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
