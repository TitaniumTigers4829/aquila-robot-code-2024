// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedFar3Note extends SequentialCommandGroup {
  /** Creates a new RedUnderStage4note. */
  public RedFar3Note(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(15.78876781463623, 4.516969680786133, Rotation2d.fromRadians(-2.150203189054567)))),
      // fender shot
      // while intaking:
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "redfar3note 1", false),
      // shoot
      // while intaking:
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "redfar3note 2", false)
      // shoot
    );
  }
}
