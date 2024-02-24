package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private final DigitalInput noteSensor;

  private final StatusSignal<Double> leaderVelocity;
  private final StatusSignal<Double> followerVelocity;
  // private SingleLinearInterpolator speakerSpeedValues;

  private double shooterTargetRPM;
  private final MotionMagicVoltage mmRequest;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    leaderFlywheel = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_ID);
    followerFlywheel = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_ID);
    rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);
    noteSensor = new DigitalInput(ShooterConstants.SHOOTER_NOTE_SENSOR_ID);

    mmRequest = new MotionMagicVoltage(0);

    // speakerSpeedValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_SHOOT_RPMS);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;
    shooterConfig.Slot0.kS = ShooterConstants.SHOOT_S;
    shooterConfig.Slot0.kV = ShooterConstants.SHOOT_V;
    shooterConfig.Slot0.kA = ShooterConstants.SHOOT_A;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    shooterConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    shooterConfig.CurrentLimits.StatorCurrentLimit = 450;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 450;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    leaderFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    rollerMotor.getConfigurator().apply(rollerConfig);
    rollerMotor.set(0);

    leaderVelocity = leaderFlywheel.getVelocity();
    followerVelocity = followerFlywheel.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, leaderVelocity, followerVelocity);
    ParentDevice.optimizeBusUtilizationForAll(leaderFlywheel, rollerMotor, followerFlywheel);
  }

  /**
   * sets the speed of the flywheel
   * @param speed the speed (m/s) of the flywheel
   */
  public void setShooterSpeed(double speed) {

    if (speed < 0.01 ) {
      leaderFlywheel.set(0);
      followerFlywheel.set(0);
    }

    leaderFlywheel.set(speed);
    followerFlywheel.set(speed);
  }

  /**
   * sets the speed of the rollers to transfer note from tower to shooter
   * @param speed speed (m/s) of the rollers
   */
  public void setRollerSpeed(double speed) {
    if(speed < 0.3) { 
      rollerMotor.set(-1);
    }
    rollerMotor.set(speed);
  }

  /**
   * gets if a note is sensed by the beam break
   * @return true if the Digital Input is tripped 
   */
  public boolean getSensor() {
    return noteSensor.get();
  }

  /**
   * the error between the target rpm and actual rpm of the shooter
   * @return True if we are within an acceptable range (of rpm) to shoot
   */
  public boolean isShooterWithinAcceptableError() {
    return Math.abs(shooterTargetRPM - getShooterRPM()) < 2000;
  }

  public boolean isReadyToShoot(double headingError) {
    return (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_DEGREES) && isShooterWithinAcceptableError();
  }

  /**
   * sets RPM of both leader and follower flywheel motors
   * @param desiredRPM sets the rpm of the leader motor
   */
  public void setRPM(double desiredRPM) {

    shooterTargetRPM = desiredRPM;
    // SmartDashboard.putNumber("Shoter desired", shooterTargetRPM);
    // RPM --> RPS
    SmartDashboard.putNumber("targetRPM", desiredRPM);
    SmartDashboard.putNumber("error", desiredRPM - (leaderVelocity.refresh().getValueAsDouble() * 60));
    SmartDashboard.putNumber("current velocity", (leaderVelocity.refresh().getValueAsDouble() * 60));
    VelocityVoltage desiredVelocity = new VelocityVoltage(desiredRPM / 60.0);
    leaderFlywheel.setControl(desiredVelocity);
    followerFlywheel.setControl(desiredVelocity);
  }


  /**
   * sets flywheel speed (m/s) to 0
   */
  public void setFlywheelNeutral() {
    leaderFlywheel.set(0);
    followerFlywheel.set(0);
  }

  // /**
  //  * sets the shooter rpm from a lookup table of 
  //  * values and the distance to the speaker
  //  * @param distance the distance (meters) from the speaker
  //  */
  // public void setShooterRPMFromDistance(double distance) {
  //   // set local target rpm
  //   shooterTargetRPM = speakerSpeedValues.getLookupValue(distance);
  //   setRPM(shooterTargetRPM);
  // }
  
  /**
   * gets the current shooter RPM
   * @return returns the current shooter rpm as a double
   */
  public double getShooterRPM() {
    leaderVelocity.refresh();
    return leaderVelocity.getValueAsDouble() * 60;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("sensor", getSensor());
  }

}
