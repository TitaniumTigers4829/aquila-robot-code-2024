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
public class RedFiveNote extends SequentialCommandGroup {
  /** Creates a new FiveNotePath. */
  public RedFiveNote(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(15.215974807739258, 5.519354820251465, Rotation2d.fromRadians(3.141592653589793)))),
      new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ()->0, ()->0, ()->0, ()->false, ledSubsystem).withTimeout(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "Red center to note 3", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(2.8)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red 3 to shoot", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(1)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red shoot to note 2", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(1.4)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "red note 2 to shoot", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(4.5)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem)
    );
  }
}
