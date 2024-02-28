// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class ManualPivot extends Command {
  private final PivotSubsystem pivotSubsytem;
  private final DoubleSupplier speed;

  /** Creates a new SetShooterAngle. */
  public ManualPivot(PivotSubsystem pivotSubsytem, DoubleSupplier speed) {
    this.pivotSubsytem = pivotSubsytem;
    this.speed = speed;
    addRequirements(pivotSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsytem.set(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsytem.set(PivotConstants.PIVOT_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
