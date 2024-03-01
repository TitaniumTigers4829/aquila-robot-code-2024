// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red4NoteAuto extends SequentialCommandGroup {
  /** Creates a new Red4NoteAuto. */
  public Red4NoteAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red4note1st", true),
      // new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red4note2nd", false),
      // new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red4note3rd", false)
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 2", false),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 2 to shoot", false),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 3", false),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 3 to shoot", false)


    );
  }
}
