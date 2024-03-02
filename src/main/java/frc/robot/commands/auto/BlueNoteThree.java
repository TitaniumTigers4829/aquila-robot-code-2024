// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.choreo.lib.Choreo;

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
public class BlueNoteThree extends SequentialCommandGroup {
  /** Creates a new BlueSimpleTwoNote. */
  public BlueNoteThree(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {
    addCommands(
        new InstantCommand(()->driveSubsystem.resetOdometry(Choreo.getTrajectory("blue to note 1").getInitialPose())),
        new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(2.0),
        new ParallelCommandGroup(
            new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 1", true),
            new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
        ),
        new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(2.0),
        new ParallelCommandGroup(
            new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 1 to 2", false),
            new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
        ),
        new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(2.0),
        new ParallelCommandGroup(
            new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 2 to 3", false),
            new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
        ),
        new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(2.0)
    );
  }
}
