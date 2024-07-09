package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANcoder turnEncoder;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private final StatusSignal<Double> driveMotorVelocity;
  private final StatusSignal<Double> driveMotorPosition;
  private final StatusSignal<Double> turnEncoderPos;

  private final MotionMagicVoltage mmPositionRequest;
  private final VelocityVoltage velocityRequest;

  private Debouncer slipDebouncer;
  private double maxLinearSpeed;
  private double optimalSlipRatio;
  private double currentSlipRatio = 0.0;
  private boolean isSlipping = false;
  
  private String name;

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turnMotorChannel ID of the turn motor
   * @param turnEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param turnReversed is the turn motor reversed
   * @param driveReversed is the drive motor reversed
   * @param name name of the motor
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
    
    turnEncoder = new CANcoder(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    driveMotor = new TalonFX(driveMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnMotor = new TalonFX(turnMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);

    mmPositionRequest = new MotionMagicVoltage(0);
    velocityRequest = new VelocityVoltage(0);
    
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
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimit = ModuleConstants.DRIVE_STATOR_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    driveMotor.getConfigurator().apply(driveConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.Slot0.kP = ModuleConstants.TURN_P;
    turnConfig.Slot0.kI = ModuleConstants.TURN_I;
    turnConfig.Slot0.kD = ModuleConstants.TURN_D;
    turnConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = turnReversed;
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND;
    turnConfig.MotionMagic.MotionMagicAcceleration = ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotor.getConfigurator().apply(turnConfig, HardwareConstants.TIMEOUT_S);

    turnEncoderPos = turnEncoder.getAbsolutePosition();
    driveMotorPosition = driveMotor.getPosition();
    driveMotorVelocity = driveMotor.getVelocity();

    driveMotor.setPosition(0);
    turnMotor.setPosition(0);

    optimalSlipRatio = MathUtil.clamp(optimalSlipRatio, 0.01, 0.4);
    maxLinearSpeed = Math.floor(DriveConstants.MAX_SPEED_METERS_PER_SECOND * 1000) / 1000;
    slipDebouncer = new Debouncer(.2, DebounceType.kRising); // majic # lol

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, turnEncoderPos, driveMotorPosition, driveMotorVelocity);

    ParentDevice.optimizeBusUtilizationForAll(turnEncoder, driveMotor, turnMotor);
  }

  /**
     * the method comes from 1690's <a href="https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674">online software session</a>
     * gets the skidding ratio from the latest , that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the  ratio between the maximum and minimum magnitude of the "translational" part of the speed of the modules
     * 
     * @param swerveStatesMeasured the swerve states measured from the modules
     * @param swerveDriveKinematics the kinematics
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     * */
    public static double getSkiddingRatio(SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
        final double angularVelocityOmegaMeasured = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
        final double[] swerveStatesTranslationalPartMagnitudes = new double[swerveStatesMeasured.length];

        for (int i =0; i < swerveStatesMeasured.length; i++) {
            final Translation2d swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
                    swerveStatesRotationalPartAsVector = convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
                    swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
        for (double translationalSpeed:swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

    private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
        return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
    }

   private void updateSlipRatio(double wheelSpeed) { //account for inertial velocity
    // Calculate current slip ratio
    currentSlipRatio = ((wheelSpeed));

    // Check if wheel is slipping, false if disabled
    // isSlipping = slipDebouncer.calculate(currentSlipRatio > optimalSlipRatio) & isEnabled();
  }

  public boolean isSlipping() {
    return isSlipping;
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

    double speedMetersPerSecond = ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotorVelocity.getValueAsDouble();

    return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromRotations(getModuleHeading()));
  }

  /**
   * Gets the module position consisting of the distance it has traveled and the angle it is rotated.
   * @return a SwerveModulePosition object containing position and rotation
   */
  public SwerveModulePosition getPosition() {
    driveMotorPosition.refresh();
    double position = ModuleConstants.DRIVE_TO_METERS * -driveMotorPosition.getValue();
    Rotation2d rotation = Rotation2d.fromRotations(getModuleHeading());
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
     
    driveMotor.setControl(velocityRequest.withVelocity(desiredDriveRPS));
    turnMotor.setControl(mmPositionRequest.withPosition(optimizedDesiredState.angle.getRotations()));

    updateSlipRatio(optimizedDesiredState.speedMetersPerSecond);
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position in radians
   */
  public double getTurnRadians() {
    turnEncoderPos.refresh();
    return Rotation2d.fromRotations(turnEncoderPos.getValue()).getRadians();
  }

  public double getDrivePositionRadians() {
    driveMotorPosition.refresh();
    return 2.0 * Math.PI * (driveMotorPosition.getValue() / Constants.ModuleConstants.DRIVE_GEAR_RATIO);
  }

  /**
   * This is called in the periodic of DriveSubsystem
   */
  public void periodicFunction() {
  }

}
