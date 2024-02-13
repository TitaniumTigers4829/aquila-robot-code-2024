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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.extras.SingleLinearInterpolator;

public class PivotSubsystem extends SubsystemBase {
  private final CANcoder pivotEncoder;

  private final TalonFX leaderPivotMotor;
  private final TalonFX followerPivotMotor;
  private final Follower follower;

  private final SingleLinearInterpolator speakerAngleLookupValues;

  private final StatusSignal<Double> pivotPos;
  private double pivotTargetAngle;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    leaderPivotMotor = new TalonFX(PivotConstants.LEADER_PIVOT_MOTOR_ID);
    followerPivotMotor = new TalonFX(PivotConstants.FOLLOWER_PIVOT_MOTOR_ID);
    pivotEncoder = new CANcoder(PivotConstants.PIVOT_ENCODER_ID);

    speakerAngleLookupValues = new SingleLinearInterpolator(PivotConstants.SPEAKER_PIVOT_POSITION);

    follower = new Follower(leaderPivotMotor.getDeviceID(), true);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -PivotConstants.ANGLE_ZERO;
    turnEncoderConfig.MagnetSensor.SensorDirection = PivotConstants.ENCODER_REVERSED;
    pivotEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = PivotConstants.PIVOT_P;
    pivotConfig.Slot0.kI = PivotConstants.PIVOT_I;
    pivotConfig.Slot0.kD = PivotConstants.PIVOT_D;
    pivotConfig.Slot0.kG = PivotConstants.PIVOT_G;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    leaderPivotMotor.getConfigurator().apply(pivotConfig);

    pivotPos = pivotEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, pivotPos);
    ParentDevice.optimizeBusUtilizationForAll(leaderPivotMotor, followerPivotMotor, pivotEncoder);
  }

  /**
   * @return rotation of pivot in degrees
  */
  public double getRotation() {
    pivotPos.refresh();
    return pivotPos.getValueAsDouble() * 360;
  }

  /**
   * returns if the pivot is within an acceptable rotation 
   * in relation to the target position
   *@return pivot error between desired and actual state in degrees
  */
  public boolean isPivotWithinAcceptableError() {
    return Math.abs(pivotTargetAngle - getRotation()) < 2;
  }

  public void set(double output) {
    leaderPivotMotor.set(output);
    followerPivotMotor.set(-output);
  }

  /**
   * gets the target angle of the pivot motors in degrees
   * @return the target angle
   */
  public double getPivotTarget() {
    return pivotTargetAngle;
  }

  /**
   * uses distance in meters from the speaker to set the pivot angle (degrees) of the shooter
   * @param distance the distance in meters from the speaker
   */
  public void setPivotFromDistance(double distance) {
    double angle = speakerAngleLookupValues.getLookupValue(distance);
    pivotTargetAngle = angle;
    setPivot(angle);
  }

  /**
   * sets the pivot using the leader/follower motors
   * @param angle the angle (degrees) to set
   */
  public void setPivot(double angle) {
    MotionMagicVoltage output = new MotionMagicVoltage(angle / 360.0);
    pivotTargetAngle = angle;
    leaderPivotMotor.setControl(output);
    followerPivotMotor.setControl(follower);
  }

  /**
   * sets the angle (degrees) of the pivot motors to a neutral position
   */
  public void setPivotMotorToNeutral() {
    setPivot(PivotConstants.PIVOT_NEUTRAL_ANGLE);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("pivot pos", pivotPos.refresh().getValueAsDouble());
  }
}
