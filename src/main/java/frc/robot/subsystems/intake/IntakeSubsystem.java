package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX leftIntakeMotor;
  private final TalonFX rightIntakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID);
    rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID);
    
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

  @Override
  public void periodic() {
  }
}