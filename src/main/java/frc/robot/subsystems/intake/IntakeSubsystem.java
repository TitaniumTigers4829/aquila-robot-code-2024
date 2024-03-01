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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final TalonFX otbPivotMotor; // otb = over the bumper
  private final TalonFX otbIntakeMotor;

  private final StatusSignal<Double> otbPos;
  private final StatusSignal<Double> otbPivotVelocity;
  private double intakeTargetAngle;

  private final MotionMagicVoltage mmRequest;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    otbPivotMotor = new TalonFX(IntakeConstants.OTB_PIVOT_ID);
    otbIntakeMotor = new TalonFX(IntakeConstants.OTB_INTAKE_ID);
    mmRequest = new MotionMagicVoltage(0);

    TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration();
    intakePivotConfig.Slot0.kP = IntakeConstants.INTAKE_P;
    intakePivotConfig.Slot0.kI = IntakeConstants.INTAKE_I;
    intakePivotConfig.Slot0.kD = IntakeConstants.INTAKE_D;
    intakePivotConfig.Slot0.kG = IntakeConstants.INTAKE_G;

    intakePivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    intakePivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.MM_ACCELERATION;
    intakePivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MM_VELOCITY;

    intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakePivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    intakePivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    
    // TODO: tune
    // intakePivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.MAX_OVERTHEBUMPER_ANGLE;
    // intakePivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_OVERTHEBUMPER_ANGLE;
    // intakePivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // intakePivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    intakePivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakePivotConfig.Feedback.FeedbackRotorOffset = IntakeConstants.OTB_ROTOR_OFFSET;

    otbPivotMotor.getConfigurator().apply(intakePivotConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);

    otbPivotVelocity = otbPivotMotor.getVelocity();
    otbPos = otbPivotMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, otbPivotVelocity, otbPos);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, otbPivotMotor, otbIntakeMotor);
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

  public void setIntakeAngle(double angle) {
    intakeTargetAngle = angle;
    otbPivotMotor.setControl(mmRequest.withPosition(angle));
  }

  public void setOTBIntakeSpeed() {
    otbIntakeMotor.set(IntakeConstants.INTAKE_OVERTHEBUMPER_SPEED);
  }

  public double getRotation() {
    otbPos.refresh();
    return otbPos.getValueAsDouble();
  }
  public boolean isIntakeWithinAcceptableError() {
    return Math.abs(intakeTargetAngle - getRotation()) < IntakeConstants.INTAKE_PIVOT_ACCEPTABLE_ERROR;
  }

  public void setPivotSpeed(double speed) {
    otbPivotMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakePos", getRotation());
  }
}