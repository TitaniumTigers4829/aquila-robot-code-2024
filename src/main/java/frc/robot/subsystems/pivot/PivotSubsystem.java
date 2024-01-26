// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.extras.SingleLinearInterpolator;

public class PivotSubsystem extends SubsystemBase {
  private final CANcoder pivotEncoder;

  private final TalonFX leaderPivotMotor;
  private final TalonFX followerPivotMotor;

  private final SingleLinearInterpolator speakerAngleLookupValues;

  private final StatusSignal<Double> pivotSpeakerPosition;

  private double pivotMotorTargetPosition;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    leaderPivotMotor = new TalonFX(PivotConstants.LEADER_PIVOT_MOTOR_ID);
    followerPivotMotor = new TalonFX(PivotConstants.FOLLOWER_PIVOT_MOTOR_ID);
    pivotEncoder = new CANcoder(PivotConstants.PIVOT_ENCODER_ID);

    speakerAngleLookupValues = new SingleLinearInterpolator(PivotConstants.SPEAKER_PIVOT_POSITION);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -PivotConstants.ANGLE_ZERO;
    turnEncoderConfig.MagnetSensor.SensorDirection = PivotConstants.ENCODER_REVERSED;
    pivotEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = PivotConstants.PIVOT_P;
    pivotConfig.Slot0.kI = PivotConstants.PIVOT_I;
    pivotConfig.Slot0.kD = PivotConstants.PIVOT_D;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    leaderPivotMotor.getConfigurator().apply(pivotConfig);

    pivotSpeakerPosition = pivotEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(250, pivotSpeakerPosition);
    pivotEncoder.optimizeBusUtilization();
  }

  public double getRotation() {
    pivotSpeakerPosition.refresh();
    return pivotSpeakerPosition.getValueAsDouble() * 360;
  }

  public boolean isPivotWithinAcceptableError() {
    return Math.abs(pivotMotorTargetPosition - getRotation()) < 2;
  }

  public void setShooterPivotFromDistance(double distance) {
    double angle = speakerAngleLookupValues.getLookupValue(distance);
    setShooterPivot(angle);
  }

  public void setShooterPivot(double angle) {
    pivotMotorTargetPosition = angle * 360;
    MotionMagicVoltage output = new MotionMagicVoltage(angle / 360.0);
    leaderPivotMotor.setControl(output);
    Follower follower = new Follower(leaderPivotMotor.getDeviceID(), true);
    followerPivotMotor.setControl(follower);
  }

  public void setPivotMotorToNeutral() {
    leaderPivotMotor.set(0);
    followerPivotMotor.set(0);
  }
  
  @Override
  public void periodic() {
  }
}
