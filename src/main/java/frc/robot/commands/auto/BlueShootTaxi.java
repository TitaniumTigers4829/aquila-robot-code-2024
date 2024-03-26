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
public class BlueShootTaxi extends SequentialCommandGroup {
  /** Creates a new BlueSimpleTwoNote. */
  public BlueShootTaxi(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {

    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(0.7696159482002258, 4.439030170440674, Rotation2d.fromRadians(-1.048231168606524)))),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, visionSubsystem, leds).withTimeout(5.0),
      new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "bluesimple", false)
    );
  }
}
