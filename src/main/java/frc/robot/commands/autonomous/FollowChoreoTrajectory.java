// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowChoreoTrajectory extends DriveCommandBase {
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  Command controllerCommand;
  ChoreoTrajectory trajectory;

  /** Creates a new FollowChoreoTrajectory. */
  public FollowChoreoTrajectory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, String trajectoryName) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    trajectory = Choreo.getTrajectory(trajectoryName);

    driveSubsystem.resetOdometry(trajectory.getInitialPose());

    controllerCommand = Choreo.choreoSwerveCommand(
      trajectory,
      driveSubsystem::getPose, 
      new PIDController(TrajectoryConstants.REALTIME_TRANSLATION_CONTROLLER_P, 0, 0), 
      new PIDController(TrajectoryConstants.REALTIME_TRANSLATION_CONTROLLER_P, 0, 0), 
      new PIDController(TrajectoryConstants.REALTIME_THETA_CONTROLLER_P, 0, 0), 
      (ChassisSpeeds speeds) -> driveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false),
        ()->false,
        driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}