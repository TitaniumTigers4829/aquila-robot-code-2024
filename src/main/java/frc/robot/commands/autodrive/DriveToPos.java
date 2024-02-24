// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveToPos extends DriveCommandBase {
  private DriveSubsystem driveSubsystem;
  private BooleanSupplier isFinished;
  private Command controllerCommand;
  private Pose2d endPose;

  // TODO: might not work, might just use NewDriveToPos.java instead
  /** Creates a new DriveToAmp. */
  public DriveToPos(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished, Pose2d finalPose) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.isFinished = isFinished;
    this.endPose = finalPose;
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<PathPoint> pathPoints = new ArrayList<PathPoint>();
    pathPoints.add(new PathPoint(driveSubsystem.getPose().getTranslation()));

    pathPoints.add(new PathPoint(endPose.getTranslation(), new RotationTarget(0, endPose.getRotation())));

    Rotation2d startRotation = driveSubsystem.getRotation2d();
    PathPlannerPath path = PathPlannerPath.fromPathPoints(pathPoints, TrajectoryConstants.PATH_CONSTRAINTS, new GoalEndState(0, endPose.getRotation()));
    PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), startRotation);

    controllerCommand = new RealTimeSwerveControllerCommand(
      trajectory, 
      SmarterDashboardRegistry::getPose, 
      (ChassisSpeeds speeds) -> driveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false), 
      isFinished, 
      endPose, 
      driveSubsystem
    );
    controllerCommand.schedule();
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    
    // TODO: LEDs
    // if (Math.abs(endPose.getX() - driveSubsystem.getPose().getX()) < TrajectoryConstants.X_TOLERANCE
    //   && Math.abs(endPose.getY() - driveSubsystem.getPose().getY()) < TrajectoryConstants.Y_TOLERANCE
    //   && Math.abs(endPose.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees()) < TrajectoryConstants.THETA_TOLERANCE) {
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}