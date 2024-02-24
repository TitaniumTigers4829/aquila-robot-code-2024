// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootWithJoystick extends Command {
  /** Creates a new ShootWithJoystick. */
  private final ShooterSubsystem shooterSubsystem;
  private DoubleSupplier speed;
  // private BooleanSupplier isManual;

  public ShootWithJoystick(ShooterSubsystem shooterSubsystem, DoubleSupplier speed) {
    this.shooterSubsystem = shooterSubsystem;
    // this.isManual = isManual;
    this.speed = speed;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (isManual.getAsBoolean()) {
    //   if (Math.abs(speed.getAsDouble()) > 0.01) {
      shooterSubsystem.setShooterSpeed(speed.getAsDouble());    
    //   } 
    //   else {
    //     shooterSubsystem.setShooterSpeed(0);
    //   }
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


