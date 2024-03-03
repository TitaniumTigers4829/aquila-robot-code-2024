// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autodrive;


// import java.util.Optional;

// import com.choreo.lib.ChoreoTrajectory;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.TrajectoryConstants;
// import frc.robot.commands.auto.FollowChoreoTrajectory;
// import frc.robot.commands.drive.DriveCommandBase;
// import frc.robot.subsystems.swerve.DriveSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem;

// public class DriveToAmp extends DriveCommandBase {
//   private final DriveSubsystem driveSubsystem;
//   private final VisionSubsystem visionSubsystem;

//   // private final BooleanSupplier isFinished;
//   private final double finalX, finalY, finalRot;
//   private boolean isRed = false;
//   private PathPlannerPath ampPos;

//   Command controllerCommand;

//   /** Creates a new NewDriveToPos. */
//   public DriveToAmp(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double finalX, double finalY, double finalRot) {
//     super(driveSubsystem, visionSubsystem);
//     this.driveSubsystem = driveSubsystem;
//     this.visionSubsystem = visionSubsystem;
//     this.finalX = finalX;
//     this.finalY = finalY;
//     this.finalRot = finalRot;
//     // this.isFinished = isFinished;
//     addRequirements(driveSubsystem, visionSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Optional<Alliance> alliance = DriverStation.getAlliance();
//     //if alliance is detected
//     if (alliance.isPresent()) {
//       //and if it's red, we're red
//       isRed = alliance.get() == Alliance.Red;
//     } else {
//       //otherwise default to blue alliance
//       isRed = true;
//     }
//     // ampPos = isRed ? new Translation2d(FieldConstants.RED_AMP_SHOOT_X, FieldConstants.RED_AMP_SHOOT_Y) : new Translation2d(FieldConstants.BLUE_AMP_SHOOT_X, FieldConstants.BLUE_AMP_SHOOT_Y);

//      ampPos = isRed ? PathPlannerPath.fromChoreoTrajectory("RedAmpTraj") : PathPlannerPath.fromChoreoTrajectory("BlueAmpTraj");

//     // // TODO: rotation?
//     // Pose2d endPose = new Pose2d(finalX, finalY, new Rotation2d());

//     controllerCommand = AutoBuilder.pathfindThenFollowPath(
//       ampPos,
//       TrajectoryConstants.PATH_CONSTRAINTS,
//       0.0
//     );

//     controllerCommand.schedule();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     super.execute();
//        // TODO: LEDs
//     // if (Math.abs(endPose.getX() - driveSubsystem.getPose().getX()) < TrajectoryConstants.X_TOLERANCE
//     //   && Math.abs(endPose.getY() - driveSubsystem.getPose().getY()) < TrajectoryConstants.Y_TOLERANCE
//     //   && Math.abs(endPose.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees()) < TrajectoryConstants.THETA_TOLERANCE) {
//     // }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     controllerCommand.cancel();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return controllerCommand.isFinished(); 
//     // || isFinished.getAsBoolean();
//   }
// }