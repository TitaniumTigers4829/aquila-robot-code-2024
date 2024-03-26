// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ManualShoot extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsytem;

  private final DoubleSupplier speed;
  private final BooleanSupplier spinFlywheels, spinRollers;

  /** Creates a new SetShooterAngle. */
  public ManualShoot(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsytem, DoubleSupplier speed, BooleanSupplier spinFlywheels, BooleanSupplier spinRollers) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsytem = pivotSubsytem;
    this.speed = speed;
    this.spinFlywheels = spinFlywheels;
    this.spinRollers = spinRollers;
    addRequirements(shooterSubsystem, pivotSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsytem.setPivotSpeed(speed.getAsDouble());
    // shooterSubsystem.setSpeed(spinFlywheels.getAsBoolean() ? 1 : 0);
    shooterSubsystem.setRPM(spinFlywheels.getAsBoolean() ? ShooterConstants.SHOOT_SPEAKER_RPM : 0);
    shooterSubsystem.setRollerSpeed(spinRollers.getAsBoolean() ? ShooterConstants.ROLLER_SHOOT_SPEED : 0);
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    pivotSubsytem.setPivotSpeed(PivotConstants.PIVOT_NEUTRAL_SPEED);
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
