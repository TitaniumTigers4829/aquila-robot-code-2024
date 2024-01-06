// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;
  }

  public void shootSpeaker() {
  }

  public void shootAmp() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
