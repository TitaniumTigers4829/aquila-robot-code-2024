// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeAuto;
import frc.robot.commands.shooter.SubwooferShot;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueJackFiveNote extends SequentialCommandGroup {
  /** Creates a new BlueJackFiveNote. */
  public BlueJackFiveNote(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(1.3980597257614136, 5.493067741394043, Rotation2d.fromRadians(0.0)))),
      new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, () -> 0, () -> 0, () -> 0, () -> false, leds),
      new ParallelCommandGroup(
          new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue center speaker to 1", false),
          new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds),
      new ParallelCommandGroup(
          new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue shoot 1 to 2", false),
          new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds),
      new ParallelCommandGroup(
          new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue shoot 2 to 3", false),
          new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds),
      new ParallelCommandGroup(
          new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue shoot 3 to 8", false),
          new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds)
    );
  }
}