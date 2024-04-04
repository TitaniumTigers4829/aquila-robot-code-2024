// TigerLib 2024

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.interpolators.MultiLinearInterpolator;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public abstract class DriveCommandBase extends Command {

  private final MultiLinearInterpolator oneAprilTagLookupTable = 
    new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable = 
    new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  Translation2d middleField = new Translation2d(8.29, 4.13);

  private final VisionSubsystem visionSubsystem;
  private final DriveSubsystem driveSubsystem;

  private double lastTimeStampSeconds = 0;
  // private int ticksAfterSeeing = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   */
  public DriveCommandBase(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    // It is important that you do addRequirements(driveSubsystem, visionSubsystem) in whatever command extends this
  }

  @Override
  public void execute() {
    // Updates the pose estimator using the swerve modules
    driveSubsystem.addPoseEstimatorSwerveMeasurement();

    // Updates the robot's odometry with april tags
    double currentTimeStampSeconds = lastTimeStampSeconds;

    if (visionSubsystem.canSeeAprilTags()) {
      // ticksAfterSeeing++;
      currentTimeStampSeconds = visionSubsystem.getTimeStampSeconds();

      double distanceFromClosestAprilTag = visionSubsystem.getDistanceFromClosestAprilTag();
      // Sets the pose estimator confidence in vision based off of number of april tags and distance
      if (visionSubsystem.getNumberOfAprilTags() == 1) {
        double[] standardDeviations = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        driveSubsystem.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      } else if (visionSubsystem.getNumberOfAprilTags() > 1) {
        double[] standardDeviations = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
        driveSubsystem.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
      }

      // Only updates the pose estimator if the limelight pose is new and reliable
      // Pose2d limelightVisionMeasurement = visionSubsystem.getPoseFromAprilTags();
      // SmartDashboard.putString("llodometry", limelightVisionMeasurement.toString());
      // if () {
        SmartDashboard.putNumber("cuurent Timp Stamp", currentTimeStampSeconds);
        SmartDashboard.putNumber("last time stamp", lastTimeStampSeconds);
      Pose2d limelightVisionMeasurement = visionSubsystem.getPoseFromAprilTags();
      // TODO: ZUNTUE: THIS IS THE CODE THAT CHECKS IF THE MEASUREMENT IS VALID
      // if (currentTimeStampSeconds > lastTimeStampSeconds) {
      //   driveSubsystem.addPoseEstimatorVisionMeasurement(limelightVisionMeasurement, Timer.getFPGATimestamp() - visionSubsystem.getLatencySeconds());
      //   SmartDashboard.putString("llodometry", limelightVisionMeasurement.toString());
      // }
    //   // }
    }
    //  else {
    //   // ticksAfterSeeing = 0;
    // }

    lastTimeStampSeconds = currentTimeStampSeconds;
  }

}