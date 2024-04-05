// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueLeftSideThreeNote extends SequentialCommandGroup {
  /** Creates a new BlueLeftSideThreeNote. */
  public BlueLeftSideThreeNote(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(0.48448625206947327, 2.9954917430877686, Rotation2d.fromRadians(0.0)))),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue side to stage", false),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds),
      new SequentialCommandGroup(
          new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue stage to 6", false),
          new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds).withTimeout(1.9)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds)
    );
  }
}
