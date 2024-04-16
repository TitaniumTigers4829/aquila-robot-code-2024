package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootAmp extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem leds;
  private final BooleanSupplier shoot;
  
  /** Creates a new ShootSpeaker. */
  public ShootAmp(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds, BooleanSupplier shoot) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leds = leds;
    this.shoot = shoot;
    addRequirements(shooterSubsystem, pivotSubsystem, leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.SHOOT_AMP_ANGLE);
    shooterSubsystem.setRPM(ShooterConstants.SHOOT_AMP_RPM);

      if (shoot.getAsBoolean()) {
        shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
        leds.setProcess(LEDProcess.SHOOT);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    leds.setProcess(LEDProcess.DEFAULT);
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
