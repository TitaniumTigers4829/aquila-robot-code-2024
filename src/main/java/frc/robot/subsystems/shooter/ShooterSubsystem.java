package frc.robot.subsystems.shooter;

import java.io.ObjectInputFilter.Status;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.extras.SingleLinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leaderFlywheel;
  private final TalonFX followerFlywheel;
  private final TalonFX rollerMotor;

  private final StatusSignal<Double> leaderShooterVelocity;
  private final StatusSignal<Double> followerShooterVelocity;
  private Follower follower;
  private SingleLinearInterpolator speakerSpeedValues;

  private double shooterTargetRPM;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    leaderFlywheel = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_ID);
    followerFlywheel = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_ID);
    rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);

    follower = new Follower(leaderFlywheel.getDeviceID(), false);

    speakerSpeedValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_SHOOT_RPMS);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;

    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    shooterConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    leaderFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    followerFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerMotor.getConfigurator().apply(rollerConfig);

    leaderShooterVelocity = leaderFlywheel.getVelocity();
    followerShooterVelocity = followerFlywheel.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, leaderShooterVelocity, followerShooterVelocity);
    ParentDevice.optimizeBusUtilizationForAll(leaderFlywheel, rollerMotor, followerFlywheel);
  }

  /**
   * sets the speed of the flywheel
   * @param speed the speed (m/s) of the flywheel
   */
  public void setShooterSpeed(double speed) {
    leaderFlywheel.set(speed);
    followerFlywheel.set(-speed);
    SmartDashboard.putNumber("follower speed", followerFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("leader", leaderFlywheel.getVelocity().getValueAsDouble());
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
   * @return True if we are within an acceptable range (of rpm) to shoot
   */
  public boolean isShooterWithinAcceptableError() {
    return Math.abs(shooterTargetRPM - getShooterRPM()) < 20;
  }

  public boolean isReadyToShoot(double headingError) {
    return (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_DEGREES) && isShooterWithinAcceptableError();
  }

  /**
   * sets RPM of both leader and follower flywheel motors
   * @param leaderRPM sets the rpm of the leader motor
   */
  public void setRPM(double leaderRPM) {
    shooterTargetRPM = leaderRPM;
    // RPM --> RPS
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
   * sets the shooter rpm from a lookup table of 
   * values and the distance to the speaker
   * @param distance the distance (meters) from the speaker
   */
  public void setShooterRPMFromDistance(double distance) {
    // set local target rpm
    shooterTargetRPM = speakerSpeedValues.getLookupValue(distance);
    setRPM(shooterTargetRPM);
  }
  
  /**
   * gets the current shooter RPM
   * @return returns the current shooter rpm as a double
   */
  public double getShooterRPM() {
    leaderShooterVelocity.refresh();
    return leaderShooterVelocity.getValueAsDouble() * 60;
  }
  
  @Override
  public void periodic() {
  }

}
