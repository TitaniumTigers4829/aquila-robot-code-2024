// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueSixNoteAuto extends SequentialCommandGroup {
  /** Creates a new BlueSixNoteAuto. */
  public BlueSixNoteAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {

    addCommands(
      new FollowPathAndShoot(driveSubsystem, visionSubsystem, pivotSubsystem, shooterSubsystem, "blue 4 note start").withTimeout(5),
      new ParallelRaceGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 3 to note 4").withTimeout(3),
        new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(2),
      new ParallelRaceGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 5").withTimeout(3),
        new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(2)
    );
  }
}
