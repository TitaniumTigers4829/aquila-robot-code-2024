// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ExampleConstants;
import frc.robot.Constants.HardwareConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX motor;
  private final DoubleSolenoid solenoid;


  public ExampleSubsystem() {
    motor = new TalonFX(0, HardwareConstants.RIO_CAN_BUS_STRING);


    solenoid = new DoubleSolenoid(HardwareConstants.PNEUMATICS_MODULE_TYPE, ExampleConstants.SOLENOID_FORWARD, ExampleConstants.SOLENOID_BACKWARD);

  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public void setMotorFullPowerIn() {
    motor.set(1);
  }

  public void setMotorFullPowerOut() {
    motor.set(-1);
  }

  public void setCustomPower(double customPower) {
    motor.set(customPower);
  }

  public void retractSolenoid() {
    solenoid.set(ExampleConstants.SOLENOID_REVERSE_VALUE);
  }

  public void extendSolenoid() {
    solenoid.set(ExampleConstants.SOLENOID_FORWARD_VALUE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
