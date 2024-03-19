package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final LaserCan intakeLc;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    intakeLc = new LaserCan(0);
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
  public boolean IntakeHasNote(){
    return intakeLc.getMeasurement().distance_mm < IntakeConstants.LASERCAN_NOTE_DETECTION_THRESHOLD;
  }
  // public void IntakeLEDs(){
  //   intakeLc = new LaserCan(0);
    // if (sensor senses note){
  //   Led = yellow;
  // }
    // if (shooter senses note){
    //   LED = Green;
    // }
    // if (intaking note){
    //   LED = Purple;
    // }
  @Override
  public void periodic() {
  }
}
