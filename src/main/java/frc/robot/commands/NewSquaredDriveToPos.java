// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class NewSquaredDriveToPos extends DriveCommandBase {
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;

  BooleanSupplier isFinished;
  double finalX, finalY, finalRot;

  Command controllerCommand;

  /** Creates a new NewDriveToPos. */
  public NewSquaredDriveToPos(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished, double finalX, double finalY, double finalRot) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.finalX = finalX;
    this.finalY = finalY;
    this.finalRot = finalRot;

    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.zeroHeading();
    driveSubsystem.resetOdometry(new Pose2d());

    Pose2d endPose = new Pose2d(finalX, finalY, Rotation2d.fromDegrees(finalRot));

    controllerCommand = AutoBuilder.pathfindToPose(
      endPose,
      TrajectoryConstants.PATH_CONSTRAINTS,
      0.0,
      0.0
    );

    controllerCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished() || isFinished.getAsBoolean();
  }
}