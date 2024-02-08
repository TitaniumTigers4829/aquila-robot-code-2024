// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FollowChoreoTrajectory extends DriveCommandBase {
  
  private final DriveSubsystem driveSubsystem;
  private final Command controllerCommand;

  /**
   * Follows a trajectory made with Choreo
   * @param driveSubsystem instance of the drive subsystem
   * @param visionSubsystem instance of the vision subsystem
   * @param trajectoryName name of the path excluding the .chor and the directory path
   */
  public FollowChoreoTrajectory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, String trajectoryName) {
    super(driveSubsystem, visionSubsystem);
    controllerCommand = Choreo.choreoSwerveCommand(
      Choreo.getTrajectory(trajectoryName),
      driveSubsystem::getPose, 
      new PIDController(TrajectoryConstants.DEPLOYED_TRANSLATION_CONTROLLER_P, 0, 0), 
      new PIDController(TrajectoryConstants.DEPLOYED_TRANSLATION_CONTROLLER_P, 0, 0), 
      new PIDController(TrajectoryConstants.DEPLOYED_THETA_CONTROLLER_P, 0, 0), 
      (ChassisSpeeds speeds) -> driveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false),
      ()->false,
      driveSubsystem
    );
    addRequirements(visionSubsystem);
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
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}
