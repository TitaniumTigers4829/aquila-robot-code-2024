// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TestShooter extends Command {
  private ShooterSubsystem shooterSubsystem;
  /** Creates a new TestShooter. */
  public TestShooter(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  @Override
  public void execute () {

    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
  }
  public boolean isReadyToShoot() {
    return shooterSubsystem.isShooterWithinAcceptableError();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
