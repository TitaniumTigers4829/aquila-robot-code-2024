// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueNoteSeven2 extends SequentialCommandGroup {
  /** Creates a new BlueNoteEight1. */
  public BlueNoteSeven2(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 8", false).withTimeout(4.2),
        new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(2.5),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 9", false).withTimeout(4.2),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(2.5)
    );
  }
}
