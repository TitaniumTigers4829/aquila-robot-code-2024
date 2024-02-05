package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.extras.SingleLinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {
  private final DigitalInput noteSensor;
  private final TalonFX leaderFlywheel;
  private final TalonFX followerFlywheel;
  private final TalonFX rollerMotor;

  private StatusSignal<Double> shooterVelocity;
  private Follower follower;
  private SingleLinearInterpolator speakerSpeedValues;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    leaderFlywheel = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_ID);
    followerFlywheel = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_ID);
    rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);
    noteSensor = new DigitalInput(ShooterConstants.SHOOTER_NOTE_SENSOR_ID);

    follower = new Follower(leaderFlywheel.getDeviceID(), true);

    speakerSpeedValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_SHOOT_RPMS);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    leaderFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerMotor.getConfigurator().apply(rollerConfig);

    shooterVelocity = leaderFlywheel.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, shooterVelocity);
    ParentDevice.optimizeBusUtilizationForAll(leaderFlywheel, rollerMotor, followerFlywheel);
  }

  /**
   * sets the speed of the flywheel
   * @param speed the speed (m/s) of the flywheel
   */
  public void setShooterSpeed(double speed) {
    leaderFlywheel.set(speed);
    followerFlywheel.setControl(follower);
  }

  /**
   * sets the speed of the rollers to transfer note from tower to shooter
   * @param speed speed (m/s) of the rollers
   */
  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  /**
   * the error between the target rpm and actual rpm of the shooter
   * @param distance the distance (meters) to target
   * @return True if we are within an acceptable range (of rpm) to shoot
   */
  public boolean isShooterWithinAcceptableError(double distance) {
    return Math.abs(getShooterTargetRPM(distance) - getShooterRPM()) < 20;
  }

  /**
   * sets RPM of both leader and follower flywheel motors
   * @param leaderRPM sets the rpm of the leader motor
   */
  public void setRPM(double leaderRPM) {
    VelocityVoltage leaderSpeed = new VelocityVoltage(leaderRPM / 60.0);
    leaderFlywheel.setControl(leaderSpeed);
    followerFlywheel.setControl(follower);
  }


  /**
   * sets flywheel speed (m/s) to 0
   */
  public void setFlywheelNeutral() {
    leaderFlywheel.set(0);
    followerFlywheel.setControl(follower);
  }

  /**
   * Gets the sensor in the shooter pivot
   * @return True if there is not a note in the tower
   */
  public boolean getSensor() {
    return noteSensor.get();
  }

  /**
   * sets the shooter rpm from a lookup table of 
   * values and the distance to the speaker
   * @param distance the distance (meters) from the speaker
   */
  public void setShooterRPMFromDistance(double distance) {
    double rpm = speakerSpeedValues.getLookupValue(distance);
    setRPM(rpm);
  }
  
  /**
   * gets the current shooter RPM
   * @return returns the current shooter rpm as a double
   */
  public double getShooterRPM() {
    shooterVelocity.refresh();
    return shooterVelocity.getValueAsDouble() * 60;
  }
  
  /**
   * gets the target RPM of the shooter based on distance from the speaker
   * @param distance the distance (meters) from the speaker
   * @return the target rpm
   */
  public double getShooterTargetRPM(double distance) {
    double targetRPM = speakerSpeedValues.getLookupValue(distance);
    return targetRPM;
  }
  
  @Override
  public void periodic() {
  }

}
