package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.extras.interpolators.SingleLinearInterpolator;

public class PivotSubsystem extends SubsystemBase {

  private final CANcoder pivotEncoder;
  private final TalonFX leaderPivotMotor;
  private final TalonFX followerPivotMotor;

  private final MotionMagicVoltage mmPositionRequest;

  private final SingleLinearInterpolator speakerAngleLookupValues;
  private final SingleLinearInterpolator passAngleLookupValues;

  private final StatusSignal<Double> pivotPos;
  private double pivotTargetAngle;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    leaderPivotMotor = new TalonFX(PivotConstants.LEADER_PIVOT_MOTOR_ID);
    followerPivotMotor = new TalonFX(PivotConstants.FOLLOWER_PIVOT_MOTOR_ID);
    pivotEncoder = new CANcoder(PivotConstants.PIVOT_ENCODER_ID);

    mmPositionRequest = new MotionMagicVoltage(0);

    speakerAngleLookupValues = new SingleLinearInterpolator(PivotConstants.SPEAKER_PIVOT_POSITION);
    passAngleLookupValues = new SingleLinearInterpolator(PivotConstants.PASS_PIVOT_POSITION);

    CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
    pivotEncoderConfig.MagnetSensor.MagnetOffset = -PivotConstants.ANGLE_ZERO;
    pivotEncoderConfig.MagnetSensor.SensorDirection = PivotConstants.ENCODER_REVERSED;
    pivotEncoder.getConfigurator().apply(pivotEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = PivotConstants.PIVOT_P;
    pivotConfig.Slot0.kI = PivotConstants.PIVOT_I;
    pivotConfig.Slot0.kD = PivotConstants.PIVOT_D;
    pivotConfig.Slot0.kG = PivotConstants.PIVOT_G;

    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    pivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;

    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.MAX_ANGLE;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.MIN_ANGLE;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leaderPivotMotor.getConfigurator().apply(pivotConfig, HardwareConstants.TIMEOUT_S);
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerPivotMotor.getConfigurator().apply(pivotConfig, HardwareConstants.TIMEOUT_S);

    pivotPos = pivotEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, pivotPos);
    ParentDevice.optimizeBusUtilizationForAll(leaderPivotMotor, followerPivotMotor, pivotEncoder);
  }

  /**
   * Gets the angle of the pivot
   * @return angle of pivot in rotations
   */
  public double getAngle() {
    pivotPos.refresh();
    return pivotPos.getValueAsDouble();
  }

  /**
   * Returns if the pivot is within an acceptable rotation 
   * in relation to the target position
   * @return pivot error between desired and actual state in rotations
   */
  public boolean isPivotWithinAcceptableError() {
    return Math.abs(pivotTargetAngle - getAngle()) < PivotConstants.PIVOT_ACCEPTABLE_ERROR;
  }

  /**
   * Sets the output of the pivot
   * @param output output value from -1.0 to 1.0
   */
  public void setPivotSpeed(double output) {
    leaderPivotMotor.set(output);
    followerPivotMotor.set(output);
  }

  /**
   * Gets the target angle of the pivot in degrees
   * @return the target angle
   */
  public double getPivotTarget() {
    return pivotTargetAngle;
  }

  /**
   * Uses distance in meters from the speaker to set the pivot angle (degrees) of the shooter
   * @param speakerDistance the distance in meters from the speaker
   */
  public void setPivotFromSpeakerDistance(double speakerDistance) {
    double speakerAngle = speakerAngleLookupValues.getLookupValue(speakerDistance);
    pivotTargetAngle = speakerAngle;
    setPivotAngle(speakerAngle);
  }

  /**
   * Uses distance in meters from the passing position to set the pivot angle (degrees) of the shooter
   * @param passDistance the distance in meters from the passing position
   */
  public void setPivotFromPassDistance(double passDistance) {
    double passAngle = passAngleLookupValues.getLookupValue(passDistance);
    pivotTargetAngle = passAngle;
    setPivotAngle(passAngle);
  }
  
  /**
   * Sets the pivot using the leader/follower motors
   * @param angle the angle (degrees) to set
   */
  public void setPivotAngle(double angle) {
    pivotTargetAngle = angle;
    leaderPivotMotor.setControl(mmPositionRequest.withPosition(angle));
    followerPivotMotor.setControl(mmPositionRequest.withPosition(angle));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("pivot pos", pivotPos.refresh().getValueAsDouble());
  }
}
