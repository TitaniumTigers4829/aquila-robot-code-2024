// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.controllers.PathFollowingController;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.PIDConstants;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.TrajectoryConstants;
// import frc.robot.commands.drive.DriveCommandBase;
// import frc.robot.subsystems.swerve.DriveSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem;

// public class FollowPathplannerPath extends DriveCommandBase {
//   DriveSubsystem driveSubsystem;
//   VisionSubsystem visionSubsystem;

//   PathPlannerTrajectory trajectory;
//   SwerveControllerCommand controllerCommand;
//   PathFollowingController controller;
//   Command followCommand;

//   boolean resetOdometry;

//   /** Creates a new FollowPathplannerPath. */
//   public FollowPathplannerPath(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, String trajectoryName, boolean resetOdometry) {
//     super(driveSubsystem, visionSubsystem);
//     this.driveSubsystem = driveSubsystem;
//     this.visionSubsystem = visionSubsystem;

//     trajectory = PathPlannerPath.fromPathFile(trajectoryName).getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0));
//     controller = new PPHolonomicDriveController(
//       new PIDConstants(TrajectoryConstants.DEPLOYED_X_CONTROLLER_P),
//       new PIDConstants(TrajectoryConstants.DEPLOYED_THETA_CONTROLLER_P),
//       TrajectoryConstants.MAX_SPEED,
//       TrajectoryConstants.DRIVE_BASE_RADIUS
//     );
//     if (resetOdometry) {
//       // controller.reset(driveSubsystem.getPose(), new ChassisSpeeds());
//       driveSubsystem.resetOdometry(trajectory.getInitialTargetHolonomicPose());
//     }
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
