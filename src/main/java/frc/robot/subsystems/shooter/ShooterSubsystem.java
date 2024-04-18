package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.extras.SmarterDashboardRegistry;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leaderFlywheel;
  private final TalonFX followerFlywheel;
  private final TalonFX rollerMotor;
  
  private final DigitalInput noteSensor;

  private final StatusSignal<Double> leaderVelocity;
  private final StatusSignal<Double> followerVelocity;

  private double shooterTargetRPM;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() { 
    leaderFlywheel = new TalonFX(ShooterConstants.LEADER_FLYWHEEL_ID);
    followerFlywheel = new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_ID);
    rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);

    noteSensor = new DigitalInput(ShooterConstants.NOTE_SENSOR_ID);
    
    velocityRequest = new VelocityVoltage(0);
    voltageRequest = new VoltageOut(0);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.SHOOT_P;
    shooterConfig.Slot0.kI = ShooterConstants.SHOOT_I;
    shooterConfig.Slot0.kD = ShooterConstants.SHOOT_D;
    shooterConfig.Slot0.kS = ShooterConstants.SHOOT_S;
    shooterConfig.Slot0.kV = ShooterConstants.SHOOT_V;
    shooterConfig.Slot0.kA = ShooterConstants.SHOOT_A;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    shooterConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_LIMIT;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.SHOOTER_STATOR_ENABLE;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.SHOOTER_SUPPLY_ENABLE;

    leaderFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    followerFlywheel.getConfigurator().apply(shooterConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    rollerMotor.getConfigurator().apply(rollerConfig, HardwareConstants.TIMEOUT_S);
    
    leaderVelocity = leaderFlywheel.getVelocity();
    followerVelocity = followerFlywheel.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, leaderVelocity, followerVelocity);
    ParentDevice.optimizeBusUtilizationForAll(leaderFlywheel, rollerMotor, followerFlywheel);
  }

  /**
   * Sets the speed of the rollers to transfer note from tower to shooter
   * @param speed 1.0 being the max speed, -1.0 being the min speed
   */
  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  /**
   * Gets if the tower has a note in it
   * @return True if there is a note
   */
  public boolean hasNote() {
    return !noteSensor.get();
  }

  /**
   * Directly sets the voltage to the shooter flywheels
   * @param volts Voltage to set
   */
  public void setVolts(double volts) {
    leaderFlywheel.setControl(voltageRequest.withOutput(volts));
    followerFlywheel.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * The error between the target rpm and actual rpm of the shooter
   * @return True if we are within an acceptable range (of rpm) to shoot
   */
  public boolean isShooterWithinAcceptableError() {
    return Math.abs(shooterTargetRPM - getShooterRPM()) < ShooterConstants.SHOOTER_ACCEPTABLE_RPM_ERROR;
  }

  /**
   * Gets if the shooter is within an acceptable rpm of the desired
   * @param headingError heading error of the drivetrain
   * @return true if shooter rpm is within and acceptable error
   */
  public boolean isReadyToShoot(double headingError) {
    return (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_RADIANS) && isShooterWithinAcceptableError();
  }

  /**
   * Sets RPM of both leader and follower flywheel motors
   * @param desiredRPM sets the rpm of the leader motor
   */
  public void setRPM(double desiredRPM) {
    shooterTargetRPM = desiredRPM;
    leaderFlywheel.setControl(velocityRequest.withVelocity(desiredRPM / 60.0));
    followerFlywheel.setControl(velocityRequest.withVelocity(desiredRPM / 60.0));
  }


  /**
   * Sets flywheel speed to 0
   */
  public void setFlywheelNeutral() {
    leaderFlywheel.set(0);
    followerFlywheel.set(0);
  }

  /**
   * Sets the flywheel speed
   * @param speed 1.0 being the max speed, -1.0 being the min speed
   */
  public void setSpeed(double speed) {
    leaderFlywheel.set(speed);
    followerFlywheel.set(speed);
  }
  
  /**
   * Gets the current shooter RPM
   * @return returns the current shooter rpm as a double
   */
  public double getShooterRPM() {
    leaderVelocity.refresh();
    return leaderVelocity.getValueAsDouble() * 60.0;
  }

 /**
 * Gets the velocity of the flywheel motors 
 * @return velocity (rot/s) of the flywheel motors
 */
  public double getFlywheelVelocity() {
    leaderVelocity.refresh();
    return leaderVelocity.getValueAsDouble();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("has note", hasNote());
    if (hasNote()) {
      SmarterDashboardRegistry.noteIn();
    } else {
      SmarterDashboardRegistry.noNote();
    }
  }
}
