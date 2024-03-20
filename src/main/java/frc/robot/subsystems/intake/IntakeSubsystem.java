package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final LaserCan intakeLc;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    intakeLc = new LaserCan(IntakeConstants.INTAKE_LASERCAN_ID);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);

    intakeMotor.optimizeBusUtilization(HardwareConstants.SIGNAL_FREQUENCY);
  }

  /**
   * Sets the intake speed
   * @param speed 1.0 being the max speed, -1.0 being the min speed
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Checks if intake has a note in it
   * @return true if there is a note
   */
  public boolean intakeHasNote(){
    return intakeLc.getMeasurement().distance_mm < IntakeConstants.INTAKE_LASERCAN_NOTE_DETECTION_THRESHOLD;
  }

  @Override
  public void periodic() {
     SmartDashboard.putBoolean("intake has note", intakeHasNote());
  }
}
