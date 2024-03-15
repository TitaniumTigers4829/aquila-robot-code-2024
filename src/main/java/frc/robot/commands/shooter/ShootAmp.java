// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

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
  
  /** Creates a new ShootSpeaker. */
  public ShootAmp(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leds = leds;
    addRequirements(shooterSubsystem);
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

    if (pivotSubsystem.isPivotWithinAcceptableError() && shooterSubsystem.isShooterWithinAcceptableError()) {
      shooterSubsystem.setTowerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
      leds.setProcess(LEDProcess.SHOOT);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
      // shooterSubsystem.setRollerSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.DEFAULT);
    shooterSubsystem.setFlywheelNeutral();
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    shooterSubsystem.setTowerSpeed(ShooterConstants.ROLLER_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
