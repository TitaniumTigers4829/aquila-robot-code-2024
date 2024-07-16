// TigerLib 2024

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.interpolators.MultiLinearInterpolator;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public abstract class DriveCommandBase extends Command {

  private final MultiLinearInterpolator oneAprilTagLookupTable = 
    new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable = 
    new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;

  private double lastTimeStampSeconds = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   */
  public DriveCommandBase(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    // It is important that you do addRequirements(driveSubsystem, visionSubsystem) in whatever
    // command extends this
  }

  @Override
  public void execute() {
    // Updates the pose estimator using the swerve modules
    if (!driveSubsystem.isSkidding() && !driveSubsystem.isCollisionDetected()) {
      driveSubsystem.addPoseEstimatorSwerveMeasurement();
    } else {
      driveSubsystem.setPoseEstimatorVisionConfidence(1, 1, 1);
    }
    // Updates the robot's odometry with april tags
    driveSubsystem.addPoseEstimatorSwerveMeasurement();
    visionSubsystem.setHeadingInfo(driveSubsystem.getOdometryRotation2d().getDegrees(), driveSubsystem.getGyroRate());
    calculatePoseFromLimelight(VisionConstants.SHOOTER_LIMELIGHT_NUMBER);
    calculatePoseFromLimelight(VisionConstants.FRONT_LEFT_LIMELIGHT_NUMBER);
    calculatePoseFromLimelight(VisionConstants.FRONT_RIGHT_LIMELIGHT_NUMBER);
  }

  public void calculatePoseFromLimelight(int limelightNumber) {
    double currentTimeStampSeconds = lastTimeStampSeconds;

    // Updates the robot's odometry with april tags
    if (visionSubsystem.canSeeAprilTags(limelightNumber)) {
      currentTimeStampSeconds = visionSubsystem.getTimeStampSeconds(limelightNumber);

      double distanceFromClosestAprilTag = visionSubsystem.getLimelightAprilTagDistance(limelightNumber);
      // Sets the pose estimator confidence in vision based off of number of april tags and distance
      if (visionSubsystem.getNumberOfAprilTags(limelightNumber) == 1) {
        double[] standardDeviations = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        driveSubsystem.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      } else if (visionSubsystem.getNumberOfAprilTags(limelightNumber) > 1) {
        double[] standardDeviations = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        driveSubsystem.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      }

      driveSubsystem.addPoseEstimatorVisionMeasurement(visionSubsystem.getPoseFromAprilTags(limelightNumber), Timer.getFPGATimestamp() - visionSubsystem.getLatencySeconds(limelightNumber));
    }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }
}
