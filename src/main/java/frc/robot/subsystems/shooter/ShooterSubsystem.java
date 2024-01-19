// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  DigitalInput limitSwitch;
  private final CANcoder pivotEncoder;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX pivotMotor;
  private double rightMotorTargetRPM;
  private double leftMotorTargetRPM;
  private double pivotMotorTargetPosition;

  StatusSignal<Double> shooterLeftMotorVelocity;
  StatusSignal<Double> shooterRightMotorVelocity;
  StatusSignal<Double> pivotSpeakerPosition;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    limitSwitch = new DigitalInput(ShooterConstants.SHOOTER_LIMIT_SWITCH_ID);
    leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
    rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);
    pivotMotor = new TalonFX(ShooterConstants.PIVOT_MOTOR_ID);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;
    leftMotor.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    rightMotor.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    
    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
    rotationConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    rotationConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    rotationConfig.Slot0.kD = ShooterConstants.SHOOT_D;
    pivotMotor.getConfigurator().apply(rotationConfig, HardwareConstants.TIMEOUT_S);

    pivotEncoder = new CANcoder(ShooterConstants.TURN_ENCODER_CHANNEL, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -ShooterConstants.ANGLE_ZERO;
    turnEncoderConfig.MagnetSensor.SensorDirection = ShooterConstants.ENCODER_REVERSED;
    pivotEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    shooterLeftMotorVelocity = leftMotor.getVelocity();
    shooterRightMotorVelocity = rightMotor.getVelocity();
    pivotSpeakerPosition = pivotEncoder.getAbsolutePosition();
  }

  public void setShooterPosition(double targetPivotSpeakerPosition) {
    MotionMagicVoltage position = new MotionMagicVoltage(targetPivotSpeakerPosition / 360);
    pivotMotor.setControl(position);
  }

  public double getRotation() {
    pivotSpeakerPosition.refresh();
    return pivotSpeakerPosition.getValueAsDouble();
  }

  public double getLeftShooterRPM() {
    shooterLeftMotorVelocity.refresh();
    return shooterLeftMotorVelocity.getValueAsDouble();
  }

  public double getRightShooterRPM() {
    shooterRightMotorVelocity.refresh();
    return shooterRightMotorVelocity.getValue();
  }

  public boolean isShooterWithinAcceptableError() {
    return Math.abs(leftMotorTargetRPM - getLeftShooterRPM()) < 20 && Math.abs(rightMotorTargetRPM - getRightShooterRPM()) < 20;
  }

  public boolean isPivotWithinAcceptableError() {
    return Math.abs(pivotMotorTargetPosition - getRotation()) < 1;
  }

  public void setRPM(double leftRPMMotor, double rightRPMMotor) {
    leftMotorTargetRPM = leftRPMMotor;
    rightMotorTargetRPM = rightRPMMotor;
    VelocityVoltage leftSpeed = new VelocityVoltage(leftRPMMotor * 60);
    VelocityVoltage rightSpeed = new VelocityVoltage(rightRPMMotor * 60);
    leftMotor.setControl(leftSpeed);
    rightMotor.setControl(rightSpeed);
  }

  public void setSpeedPivot(double pivotRPMMotor) {
    VelocityVoltage speed = new VelocityVoltage(pivotRPMMotor * 60);
    pivotMotor.setControl(speed);
  }

  public void setLeftMotorToNeutral() {
    rightMotor.set(0);
  }

  public void setRightMotorToNeutral() {
    leftMotor.set(0);
  }

  public void setPivotMotorToNeutral() {
    pivotMotor.set(0);
  }

  public boolean isLimitSwitchPressed() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

}