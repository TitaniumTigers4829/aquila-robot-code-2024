// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SpinUpForSpeaker;
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
public class RedAmpSideFourNote extends SequentialCommandGroup {
  /** Creates a new RedAmpSideFourNote. */
  public RedAmpSideFourNote(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(15.752967834472656, 6.664938449859619, Rotation2d.fromRadians(2.0491481821878663)))),
      new SpinUpForSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, null, null, null, ledSubsystem).withTimeout(0.01),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red amp side four note 1", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(2)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.7),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red amp side four note 2", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(2.8)
      ),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red amp side four note 3", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(3),
      new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem)
    );
  }
}
