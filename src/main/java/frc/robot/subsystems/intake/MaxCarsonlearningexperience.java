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

public class MaxCarsonLearningExperience extends SubsystemBase {
    
    private final TalonFX leftIntakeMotor;
    private final TalonFX rightIntakeMotor;
    private final TalonFX flapperMotor;
    private TalonFXConfiguration motorConfigs;
    
    public MaxCarsonLearningExperience() {
        leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID);
        rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID);
        flapperMotor = new TalonFX(IntakeConstants.FLAPPER_MOTOR_ID);
        motorConfigs = new TalonFXConfiguration();

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake();
        leftIntakeMotor.getConfigurator(motorConfigs);
        rightIntakeMotor.getConfigurator(motorConfigs);
        flapperMotor.getConfigurator(motorConfigs);

    public setSpeed(double speed){
        leftIntakeMotor.set(speed);
        rightIntakeMotor.set(speed);
        flapperMotor.set(speed);
    }
        

    }
} 