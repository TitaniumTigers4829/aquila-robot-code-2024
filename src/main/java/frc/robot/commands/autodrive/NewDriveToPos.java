// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class NewDriveToPos extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final BooleanSupplier isFinished;
  private final double finalX, finalY, finalRot;

  Command controllerCommand;

  /** Creates a new NewDriveToPos. */
  public NewDriveToPos(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished, double finalX, double finalY, double finalRot) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.finalX = finalX;
    this.finalY = finalY;
    this.finalRot = finalRot;
    this.isFinished = isFinished;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: doesnt work without this line:
    driveSubsystem.resetOdometry(driveSubsystem.getPose());

    // make a Pose2d object of the end pose
    Pose2d endPose = new Pose2d(finalX, finalY, Rotation2d.fromDegrees(finalRot));

    // create the following command
    controllerCommand = new PathfindHolonomic(
      endPose,
      TrajectoryConstants.PATH_CONSTRAINTS,
      0.0, // end velocity
      driveSubsystem::getPose,
      driveSubsystem::getRobotRelativeSpeeds,
      driveSubsystem::drive,
      TrajectoryConstants.PATH_FOLLOWER_CONFIG,
      0.0, // distance to travel before rotating
      driveSubsystem
    );

    controllerCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // updates the pose and whatnot
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
