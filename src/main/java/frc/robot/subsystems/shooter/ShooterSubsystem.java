// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  DigitalInput limitSwitch;
  private final CANcoder pivotEncoder;
  private final TalonFX leftMotor;
  private final TalonFX pivotMotor;
  private double leftMotorTargetRPM;
  private double pivotMotorTargetPosition;

  StatusSignal<Double> shooterLeftMotorVelocity;
  StatusSignal<Double> pivotSpeakerPosition;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    limitSwitch = new DigitalInput(ShooterConstants.SHOOTER_LIMIT_SWITCH_ID);
    leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
    pivotMotor = new TalonFX(ShooterConstants.PIVOT_MOTOR_ID);
    pivotEncoder = new CANcoder(ShooterConstants.TURN_ENCODER_CHANNEL, HardwareConstants.CANIVORE_CAN_BUS_STRING);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    leftMotor.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    
    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
    rotationConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    rotationConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    rotationConfig.Slot0.kD = ShooterConstants.SHOOT_D;

    rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    rotationConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    rotationConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotMotor.getConfigurator().apply(rotationConfig, HardwareConstants.TIMEOUT_S);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -ShooterConstants.ANGLE_ZERO;
    turnEncoderConfig.MagnetSensor.SensorDirection = ShooterConstants.ENCODER_REVERSED;

    pivotEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    shooterLeftMotorVelocity = leftMotor.getVelocity();
    pivotSpeakerPosition = pivotEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(250, shooterLeftMotorVelocity, pivotSpeakerPosition);
    leftMotor.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);
    pivotMotor.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);
    pivotEncoder.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);
  }

  public void setShooterPosition(double targetPivotSpeakerPosition) {
    pivotMotorTargetPosition = targetPivotSpeakerPosition;
    MotionMagicVoltage position = new MotionMagicVoltage(targetPivotSpeakerPosition / 360.0);
    pivotMotor.setControl(position);
  }

  public double getRotation() {
    pivotSpeakerPosition.refresh();
    return pivotSpeakerPosition.getValueAsDouble() * 360;
  }

  public double getLeftShooterRPM() {
    shooterLeftMotorVelocity.refresh();
    return shooterLeftMotorVelocity.getValueAsDouble() * 60;
  }

  public void setShooterSpeed(double speed) {
    leftMotor.set(speed);
  }

  public boolean isShooterWithinAcceptableError() {
    return Math.abs(leftMotorTargetRPM - getLeftShooterRPM()) < 20;
  }

  public boolean isPivotWithinAcceptableError() {
    return Math.abs(pivotMotorTargetPosition - getRotation()) < 2;
  }

  public void setRPM(double leftRPMMotor) {
    leftMotorTargetRPM = leftRPMMotor;
    VelocityVoltage leftSpeed = new VelocityVoltage(leftRPMMotor / 60.0);
    leftMotor.setControl(leftSpeed);
  }

  public void setLeftMotorToNeutral() {
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
    if (isLimitSwitchPressed()) {
      pivotMotor.setPosition(0);
    }    
  }

}