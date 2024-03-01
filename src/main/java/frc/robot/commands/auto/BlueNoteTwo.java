// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.pivot.PivotSubsystem;
// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.swerve.DriveSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class BlueNoteTwo extends SequentialCommandGroup {
//   /** Creates a new BlueSimpleTwoNote. */
//   public BlueNoteTwo(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(3),
//       new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 1", true).withTimeout(3),
//       new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(3),
//       new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 1 to 2", false).withTimeout(2),
//       new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem).withTimeout(3)
//     );
//   }
// }
