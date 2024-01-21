// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.DriveCommandBase;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveToAmp extends DriveCommandBase {
  private DriveSubsystem driveSubsystem;
  private BooleanSupplier isFinished;
  private Pose2d endPose;

  /** Creates a new DriveToAmp. */
  public DriveToAmp(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, BooleanSupplier isFinished) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.isFinished = isFinished;
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Translation2d> pathPoints = new ArrayList<Translation2d>();
    pathPoints.add(driveSubsystem.getPose().getTranslation());

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        pathPoints.add(new Translation2d(FieldConstants.RED_AMP_X, FieldConstants.RED_AMP_Y));
      } else {
        pathPoints.add(new Translation2d(FieldConstants.BLUE_AMP_X, FieldConstants.BLUE_AMP_Y));
      }
    } else {
      pathPoints.add(new Translation2d(FieldConstants.RED_AMP_X, FieldConstants.RED_AMP_Y));
    }

    endPose = new Pose2d(pathPoints.get(pathPoints.size() - 1), FieldConstants.AMP_ROTATION);
    Rotation2d startRotation = driveSubsystem.getRotation2d();
    PathPlannerPath path = new PathPlannerPath(pathPoints, TrajectoryConstants.PATH_CONSTRAINTS, new GoalEndState(0, FieldConstants.AMP_ROTATION));
    PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), startRotation);

    new RealTimeSwerveControllerCommand(
      trajectory, 
      driveSubsystem::getPose, 
      (ChassisSpeeds speeds) -> driveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false), 
      isFinished, 
      endPose, 
      driveSubsystem
    ).schedule();
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
    return false;
  }
}
