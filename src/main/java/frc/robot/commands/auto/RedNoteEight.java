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
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedNoteEight extends SequentialCommandGroup {
  /** Creates a new BlueSimpleTwoNote. */
  public RedNoteEight(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {
    addCommands(
        new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(15.825634956359863, 4.402584552764893, Rotation2d.fromRadians(-2.0701431295199386)))),
        new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(2),
        new ParallelCommandGroup(
            new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red to note 8", false),
            new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds).withTimeout(5)
        ),
        new ParallelCommandGroup(
            new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red note 8 to shoot", false),
            new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds).withTimeout(3)
        ),
        new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(3),
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red 8 shoot to 7", false)
    );
  }
}