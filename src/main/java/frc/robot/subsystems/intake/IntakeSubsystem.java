// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  // private final TalonFX otbPivotMotor; // otb = over the bumper
  // private final TalonFX otbMotor;

  //private final StatusSignal<Double> otbPos;
  // private final StatusSignal<Double> otbPivotVelocity;
  // private final StatusSignal<Double> otbVelocity;
  private double intakeTargetAngle;

  private final MotionMagicVoltage mmRequest;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    // otbPivotMotor = new TalonFX(IntakeConstants.INTAKE_PIVOT_MOTOR_ID);
    // otbMotor = new TalonFX(IntakeConstants.OVERTHEBUMPER_INTAKE_MOTOR_ID);
    mmRequest = new MotionMagicVoltage(0);


    // TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration();
    // intakePivotConfig.Slot0.kP = IntakeConstants.INTAKE_P;
    // intakePivotConfig.Slot0.kI = IntakeConstants.INTAKE_I;
    // intakePivotConfig.Slot0.kD = IntakeConstants.INTAKE_D;
    // intakePivotConfig.Slot0.kG = IntakeConstants.INTAKE_G;

    // intakePivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // intakePivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.MM_ACCELERATION;
    // intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MM_VELOCITY;

    // intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // intakePivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    // intakePivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    
    // intakePivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.MAX_OVERTHEBUMPER_ANGLE;
    // intakePivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_OVERTHEBUMPER_ANGLE;
    // intakePivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // intakePivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // otbPivotMotor.getConfigurator().apply(intakePivotConfig, HardwareConstants.TIMEOUT_S);
    // intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // intakeConfig.Feedback.FeedbackRotorOffset = rotor offset;
    // intakeConfig.Feedback.FeedbackRemoteSensorID = idk;

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // otbMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);
    intakeMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);

    // otbPivotVelocity = otbPivotMotor.getVelocity();
    // otbVelocity = otbMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY); //, otbVelocity, otbPivotVelocity
    ParentDevice.optimizeBusUtilizationForAll( intakeMotor); //otbPivotMotor, otbMotor,
  }

  /**
   * sets the intake speed
   */
  public void setIntakeSpeed(double speed) {
    if (speed < 0.01) { 
      intakeMotor.set(0);
    }
    intakeMotor.set(speed);
  }

  // public void setIntakeAngle(double angle) {
  //   intakeTargetAngle = angle;
  //   otbPivotMotor.setControl(mmRequest.withPosition(angle / 360.0));
  // }

  // public void setOTBIntakeSpeed() {
  //   otbMotor.set(IntakeConstants.INTAKE_OVERTHEBUMPER_SPEED);
  // }

  // public double getRotation() {
  //   otbPos.refresh();
  //   return otbPos.getValueAsDouble();
  // }
  //public boolean isIntakeWithinAcceptableError() {
  //       return Math.abs(intakeTargetAngle - getRotation()) < IntakeConstants.PIVOT_ACCEPTABLE_ERROR;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}