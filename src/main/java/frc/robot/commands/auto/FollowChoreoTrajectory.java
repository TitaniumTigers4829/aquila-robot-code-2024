// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

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
  
  private final DriveSubsystem driveSubsystem;
  private final ChoreoTrajectory traj;
  private final Command controllerCommand;
  private final boolean resetOdometry;

  /**
   * Follows a trajectory made with Choreo
   * @param driveSubsystem instance of the drive subsystem
   * @param visionSubsystem instance of the vision subsystem
   * @param trajectoryName name of the path excluding the .chor and the directory path
   */
  public FollowChoreoTrajectory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, String trajectoryName, boolean resetOdometry) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    traj = Choreo.getTrajectory(trajectoryName);
    controllerCommand = Choreo.choreoSwerveCommand(
      traj,
      driveSubsystem::getPose, 
      new PIDController(TrajectoryConstants.AUTO_TRANSLATION_P, 0.0, TrajectoryConstants.AUTO_TRANSLATION_D), 
      new PIDController(TrajectoryConstants.AUTO_TRANSLATION_P, 0.0, TrajectoryConstants.AUTO_TRANSLATION_D), 
      new PIDController(TrajectoryConstants.AUTO_THETA_P, 0, TrajectoryConstants.AUTO_THETA_D), 
      (ChassisSpeeds speeds) -> driveSubsystem.drive(speeds),
      ()->false,
      driveSubsystem
    );
    this.resetOdometry = resetOdometry;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (resetOdometry) {  
      driveSubsystem.resetOdometry(traj.getInitialPose());
    }
    controllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
    driveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}
