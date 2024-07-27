package frc.robot.commands.autodrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.extras.NoteDetector;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveToNote extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final NoteDetector noteDetector;

  private final ProfiledPIDController xTranslationController =
      new ProfiledPIDController(
          TrajectoryConstants.AUTO_ALIGN_TRANSLATIONAL_P,
          TrajectoryConstants.AUTO_ALIGN_TRANSLATIONAL_I,
          TrajectoryConstants.AUTO_ALIGN_TRANSLATIONAL_D,
          TrajectoryConstants.AUTO_ALIGN_TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController yTranslationController =
      new ProfiledPIDController(
          TrajectoryConstants.AUTO_ALIGN_TRANSLATIONAL_P,
          TrajectoryConstants.AUTO_ALIGN_TRANSLATIONAL_I,
          TrajectoryConstants.AUTO_ALIGN_TRANSLATIONAL_D,
          TrajectoryConstants.AUTO_ALIGN_TRANSLATION_CONSTRAINTS);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          TrajectoryConstants.AUTO_ALIGN_ROTATIONAL_P,
          TrajectoryConstants.AUTO_ALIGN_ROTATIONAL_I,
          TrajectoryConstants.AUTO_ALIGN_ROTATIONAL_D,
          TrajectoryConstants.AUTO_ALIGN_ROTATIONAL_CONSTRAINTS);

  private Translation2d noteRobotRelativeOffset;
  private Pose2d noteFieldRelativePose;

  /** Creates a new AutoAlignWithAmp. */
  public DriveToNote(
      DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, NoteDetector noteDetector) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.noteDetector = noteDetector;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  @Override
  public void initialize() {
    // Gets the note position relative to the robot
    noteRobotRelativeOffset =
        noteDetector.applyCameraOffset(noteDetector.getNoteRobotRelativeOffset());

    Pose2d robotPos = driveSubsystem.getPose();

    // Uses trigonometry to get angle of note and robot relative position to the field
    Rotation2d thetaTotal =
        Rotation2d.fromRadians(
            robotPos.getRotation().getRadians()
                + Math.atan(-noteRobotRelativeOffset.getX() / noteRobotRelativeOffset.getY()));

    double noteDistance =
        Math.sqrt(
            Math.pow(noteRobotRelativeOffset.getX(), 2)
                + Math.pow(noteRobotRelativeOffset.getY(), 2));

    // Gets the field position relative to the robot
    noteFieldRelativePose =
        new Pose2d(
            robotPos.getX() + noteDistance * Math.sin(Math.PI / 2 - thetaTotal.getRadians()),
            robotPos.getY() + noteDistance * Math.cos(Math.PI / 2 - thetaTotal.getRadians()),
            thetaTotal);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    super.execute();
    if (noteRobotRelativeOffset.getX() >= 0 && noteRobotRelativeOffset.getY() >= 0.0) {
      Pose2d robotPos = driveSubsystem.getPose();

      // Gets the pose error
      double xPoseError = noteFieldRelativePose.getX() - robotPos.getX();
      double yPoseError = noteFieldRelativePose.getY() - robotPos.getY();
      double thetaError =
          noteFieldRelativePose.getRotation().getRadians() - robotPos.getRotation().getRadians();

      // Uses the PID controllers to calculate the drive output
      double xOutput = deadband(xTranslationController.calculate(xPoseError, 0));
      double yOutput = deadband(yTranslationController.calculate(yPoseError, 0));
      double thetaOutput = thetaController.calculate(thetaError, 0);

      // Gets the chassis speeds for the robot using the odometry rotation (not alliance relative)
      ChassisSpeeds chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xOutput, yOutput, thetaOutput, driveSubsystem.getGyroFieldRelativeRotation2d());

      // Drives the robot towards the amp
      driveSubsystem.drive(chassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadband(double val) {
    if (Math.abs(val) < 0.05) {
      return 0.0;
    } else {
      return val;
    }
  }
}
