// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TowerIntake;
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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(()->driveSubsystem.resetOdometry(Choreo.getTrajectory("blue to note 1").getInitialPose())),
        // new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds, intakeSubsystem).withTimeout(5),
        // new ParallelRaceGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 1", true),
            // new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, leds)
        // ),
        // new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds, intakeSubsystem).withTimeout(5),
        // new ParallelRaceGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 1 to 2", false),
            // new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, leds)
        // ),
        // new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds, intakeSubsystem).withTimeout(5),
        // new ParallelRaceGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 2 to 3", false)
        //     new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, leds)
        // ),
        // new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds, intakeSubsystem).withTimeout(5)
    );
  }
}
