// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
    intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeMotor.getConfigurator().apply(intakeConfiguration);

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY);

    intakeMotor.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);
  }

  /**
   * sets the intake speed
   * @param speed the speed to set
   */
  public void setIntakeSpeed(double speed) {
    if (speed < 0.01) { 
      intakeMotor.set(0);
    }
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
