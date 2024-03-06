// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SubwooferShot;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FenderShotThenTaxi extends SequentialCommandGroup {
  /** Creates a new FenderShotThenTaxi. */
  public FenderShotThenTaxi(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    addCommands(
      new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ()->0, ()->0, ()->0, ()->false, ledSubsystem).withTimeout(3),
      new SimplyTaxi(driveSubsystem)
    );
  }
}