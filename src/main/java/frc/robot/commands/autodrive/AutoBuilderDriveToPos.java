package frc.robot.commands.autodrive;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoBuilderDriveToPos extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;

  private final double finalX, finalY;
  private final Rotation2d finalRot;

  private Command controllerCommand;

  /** Creates a new NewDriveToPos. */
  public AutoBuilderDriveToPos(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double finalX, double finalY, Rotation2d finalRot) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.finalX = finalX;
    this.finalY = finalY;
    this.finalRot = finalRot;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
    Pose2d endPose = new Pose2d(finalX, finalY, finalRot);

    controllerCommand = AutoBuilder.pathfindToPose(
      endPose,
      TrajectoryConstants.PATH_CONSTRAINTS,
      0.0,
      0.0
    );

    controllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    controllerCommand.execute();
    // TODO: LEDs
    // if (Math.abs(endPose.getX() - driveSubsystem.getPose().getX()) < TrajectoryConstants.X_TOLERANCE
    //   && Math.abs(endPose.getY() - driveSubsystem.getPose().getY()) < TrajectoryConstants.Y_TOLERANCE
    //   && Math.abs(endPose.getRotation().getDegrees() - driveSubsystem.getPose().getRotation().getDegrees()) < TrajectoryConstants.THETA_TOLERANCE) {
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished(); 
  }
}