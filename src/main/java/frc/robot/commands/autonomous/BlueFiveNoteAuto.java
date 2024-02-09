// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueFiveNoteAuto extends SequentialCommandGroup {
  /** Creates a new FourNoteAuto. */
  public BlueFiveNoteAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // TODO: parallel command groups for pickup & revving the shooter
    addCommands(
      // shoot
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 1"),
      // pickup
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 1 to shoot"),
      // shoot
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 2"),
      // pickup
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 2 to shoot"),
      // shoot
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 3"),
      // pickup
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 3 to shoot"),
      // shoot
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 4"),
      // pickup
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 4 to shoot")
      // shoot
    );
  }
}
