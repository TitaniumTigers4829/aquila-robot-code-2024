// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.extras.NoteDetector;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNote extends ParallelCommandGroup {
  /** Creates a new IntakeNote. */
  

  public IntakeNote(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, NoteDetector noteDetector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  addCommands(
      new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem), 
    new DriveToNote(driveSubsystem, visionSubsystem, noteDetector));
  }
}
