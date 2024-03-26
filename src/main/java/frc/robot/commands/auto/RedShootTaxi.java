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

public class RedShootTaxi extends SequentialCommandGroup {
  /** Creates a new RedShootTaxi. */
  public RedShootTaxi(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem leds) {

    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(15.7587890625, 4.4402618408203125, Rotation2d.fromRadians(-2.096547608393006)))),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, visionSubsystem, leds).withTimeout(5.0),
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "redsimple", false)
    );
  }
}
