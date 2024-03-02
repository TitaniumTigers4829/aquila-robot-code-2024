// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.intake.TowerIntake;
// import frc.robot.commands.shooter.ShootSpeaker;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.pivot.PivotSubsystem;
// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.swerve.DriveSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class BlueFiveNoteAuto extends SequentialCommandGroup {
//   /** Creates a new BlueSixNoteAuto. */
//   public BlueFiveNoteAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {

//     addCommands(
//       new FollowPathAndShoot(driveSubsystem, visionSubsystem, pivotSubsystem, shooterSubsystem, "blue 4 note start", true),
//       new ParallelRaceGroup(
//         new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 3 to note 4", false),
//         new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, leds)
//       ),
//       new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, visionSubsystem, pivotSubsystem)
//       );
//     }
// }