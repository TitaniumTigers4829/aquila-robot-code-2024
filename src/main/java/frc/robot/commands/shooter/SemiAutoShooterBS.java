// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SemiAutoShooterBS extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final DoubleSupplier pivotSpeed;
  private final BooleanSupplier rollerIn, rollerOut, shooterSpeed;

  /** Creates a new ManualShooterBS. */
  public SemiAutoShooterBS(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, DoubleSupplier pivotSpeed, BooleanSupplier shooterSpeed, BooleanSupplier rollerIn, BooleanSupplier rollerOut) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSpeed = shooterSpeed;
    this.pivotSpeed = pivotSpeed;
    this.rollerIn = rollerIn;
    this.rollerOut = rollerOut;
    addRequirements(shooterSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // we love ternary operators
    shooterSubsystem.setRPM(shooterSpeed.getAsBoolean() ? ShooterConstants.LEFT_SHOOT_SPEAKER_RPM : 0);
    shooterSubsystem.setRollerSpeed(rollerIn.getAsBoolean() ? ShooterConstants.ROLLER_SPEED : rollerOut.getAsBoolean() ? -1 * ShooterConstants.ROLLER_SPEED : 0);
    // pivotSubsystem.setPivotSpeed(pivotSpeed.getAsDouble() > 0.1 ? 0.4 : pivotSpeed.getAsDouble() < -0.1 ? -0.4 : 0);
    double angle = Math.atan2(Units.inchesToMeters(56), SmartDashboard.getNumber("speakerDist", 3.0));
    pivotSubsystem.setPivot(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setRollerSpeed(0);
    shooterSubsystem.setShooterSpeed(0);
    pivotSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
