package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX leftIntakeMotor;
  private final TalonFX rightIntakeMotor;
  private final TalonFX flapperMotor;

  private final DigitalInput noteSensor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID);
    rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID);
    flapperMotor = new TalonFX(IntakeConstants.FLAPPER_MOTOR_ID);
    noteSensor = new DigitalInput(IntakeConstants.NOTE_SENSOR_ID);
    
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.INTAKE_STATOR_ENABLE;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.INTAKE_SUPPLY_ENABLE;

    leftIntakeMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);
    rightIntakeMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);

    ParentDevice.optimizeBusUtilizationForAll(leftIntakeMotor, rightIntakeMotor);
  }

  /**
   * Sets the intake speed
   * @param speed 1.0 being the max speed, -1.0 being the min speed
   */
  public void setIntakeSpeed(double speed) {
    rightIntakeMotor.set(speed);
    leftIntakeMotor.set(speed);
  }

  /**
   * Gets whether or not the intake sensor detects a note
   * @return whether or not it detects a note
   */
  public boolean sensorDetectsNote() {
    // no object = HIGH = true
    return !noteSensor.get();
  }

  /**
   * Sets the flapper speed
   * @param speed 1.0 being the max speed, -1.0 being the min speed
   */
  public void setFlapperSpeed(double speed) {
    flapperMotor.set(speed);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("intake", sensorDetectsNote());
  }
}