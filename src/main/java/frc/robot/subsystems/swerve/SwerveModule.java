package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANcoder turnEncoder;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private final ProfiledPIDController turnController;
 
  StatusSignal<Double> driveMotorPosition;
  StatusSignal<Double> driveMotorVelocity;
  StatusSignal<Double> turnEncoderPos;
  StatusSignal<Double> turnMotorPos;

  private String name;

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turnMotorChannel ID of the turn motor
   * @param turnEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param driveReversed is the drive motor reversed
   */
  public SwerveModule(
    int driveMotorChannel,
    int turnMotorChannel,
    int turnEncoderChannel,
    double angleZero,
    SensorDirectionValue encoderReversed,
    InvertedValue turnReversed,
    InvertedValue driveReversed,
    String name
    ) {
    this.name = name;
    
    turnEncoder = new CANcoder(turnEncoderChannel);
    driveMotor = new TalonFX(driveMotorChannel);
    turnMotor = new TalonFX(turnMotorChannel);
    
    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -angleZero;
    turnEncoderConfig.MagnetSensor.SensorDirection = encoderReversed;
    turnEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
    driveConfig.Slot0.kA = ModuleConstants.DRIVE_A;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = driveReversed;
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    
    // TODO: current limits & optimize status signals
    driveMotor.getConfigurator().apply(driveConfig, HardwareConstants.TIMEOUT_S);

    turnController = new ProfiledPIDController(ModuleConstants.TURN_P, ModuleConstants.TURN_I, ModuleConstants.TURN_S, new Constraints(ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND, ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED));
    turnController.enableContinuousInput(-0.5, 0.5);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    // turnConfig.Slot0.kP = ModuleConstants.TURN_P;
    // turnConfig.Slot0.kI = ModuleConstants.TURN_I;
    // turnConfig.Slot0.kD = ModuleConstants.TURN_D;
    // turnConfig.Slot0.kS = ModuleConstants.TURN_S;
    // turnConfig.Slot0.kV = ModuleConstants.TURN_V;
    // turnConfig.Slot0.kA = ModuleConstants.TURN_A;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = turnReversed;
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    // turnConfig.ClosedLoopGeneral.ContinuousWrap = true; // enables continuous input
    // turnConfig.MotionMagic.MotionMagicCruiseVelocity = ModuleConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    // turnConfig.MotionMagic.MotionMagicAcceleration = ModuleConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
    // turnConfig.MotionMagic.MotionMagicJerk = 0;
    // turnConfig.DifferentialSensors.DifferentialSensorSource = DifferentialSensorSourceValue.RemoteCANcoder;
    // turnConfig.DifferentialSensors.DifferentialRemoteSensorID = turnEncoder.getDeviceID();

    // TODO: config current limits & optimize status signals & motionmagic
    turnMotor.getConfigurator().apply(turnConfig, HardwareConstants.TIMEOUT_S);

    turnEncoderPos = turnEncoder.getAbsolutePosition();
    driveMotorPosition = driveMotor.getPosition();
    driveMotorVelocity = driveMotor.getVelocity();

    driveMotor.setPosition(0);
    turnMotor.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, turnEncoderPos, driveMotorPosition, driveMotorVelocity);

    turnEncoder.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);
    driveMotor.optimizeBusUtilization(HardwareConstants.TIMEOUT_S);

  }

  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading() {
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    driveMotorVelocity.refresh();

    //double speedMetersPerSecond = ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotorVelocity.getValue();
    double speedMetersPerSecond = ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotorVelocity.getValueAsDouble();

    return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromRotations(getModuleHeading()));
  }

  // public SwerveModulePosition getPosition() {
  //   driveMotorPosition.refresh();
  //   driveMotorVelocity.refresh();

  //   double position = ModuleConstants.DRIVE_TO_METERS * driveMotor.getPosition().getValue();
  //   Rotation2d rotation = Rotation2d.fromDegrees(getCANCoderABS());

  //   SmartDashboard.putString(name, new SwerveModulePosition(position, rotation).toString());

  //   return new SwerveModulePosition(position, rotation);
  // }

  public SwerveModulePosition getPosition() {
    driveMotorPosition.refresh();
    double position = ModuleConstants.DRIVE_TO_METERS * driveMotorPosition.getValue();
    Rotation2d rotation = Rotation2d.fromRotations(getCANCoderABS());
    SmartDashboard.putString(name, new SwerveModulePosition(position, rotation).toString());
    return new SwerveModulePosition(position, rotation);
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRadians = getTurnRadians();
 
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));

    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
      driveMotor.set(0);
      turnMotor.set(0);
      return;
    }

    // Converts meters per second to rotations per second
    double desiredDriveRPS = optimizedDesiredState.speedMetersPerSecond 
     * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    VelocityVoltage driveOutput = new VelocityVoltage(desiredDriveRPS); // test this... might not work.
    driveMotor.setControl(driveOutput);

    
    double output = turnController.calculate(getModuleHeading(), optimizedDesiredState.angle.getRotations());
    turnMotor.set(output);
  }

  public double getTurnRadians() {
    // return ((2 * Math.PI) / 360) * turnEncoder.getAbsolutePosition();
    turnEncoderPos.refresh();
    return Rotation2d.fromRotations(turnEncoderPos.getValue()).getRadians();
  }

  public double getAbsolutePosition() {
    // return turnEncoder.getAbsolutePosition();
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position in rotations
   */
  public double getCANCoderABS(){
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    turnEncoder.setPosition(0);
    driveMotor.setPosition(0);
  }

  public void periodicFunction() {
  }
}