// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class ManualPivot extends Command {
  private final PivotSubsystem pivotSubsytem;
  private final DoubleSupplier speed;
  
  private DriveSubsystem driveSubsystem;
  // private Translation2d speakerPos;
  // private boolean isRed = false;

  /** Creates a new SetShooterAngle. */
  public ManualPivot(PivotSubsystem pivotSubsytem, DoubleSupplier speed) {
    this.pivotSubsytem = pivotSubsytem;
    this.speed = speed;
    addRequirements(pivotSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    // // get positions of various things
    // Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    // // distance (for speaker lookups)
    // double distance = robotPos.getDistance(speakerPos);

    // SmartDashboard.putNumber("SpeakerDist", distance);
    pivotSubsytem.setPivotSpeed(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsytem.setPivotSpeed(PivotConstants.PIVOT_NEUTRAL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
