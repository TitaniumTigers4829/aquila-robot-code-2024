// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import javax.swing.GroupLayout.SequentialGroup;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueNoteOne extends SequentialCommandGroup {
  /** Creates a new BlueSimpleTwoNote. */
  public BlueNoteOne(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {

    addCommands(
      // new InstantCommand(()->driveSubsystem.resetOdometry(Choreo.getTrajectory("blue to note 1").getInitialPose())),
      // new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(3),
      new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ()->0, ()->0, ()->false, leds).withTimeout(3),
      new TrajAndIntake(driveSubsystem, visionSubsystem, intakeSubsystem, pivotSubsystem, shooterSubsystem, leds, "blue to note 1", true)
      // new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, leds).withTimeout(3)
    );
  }
}
