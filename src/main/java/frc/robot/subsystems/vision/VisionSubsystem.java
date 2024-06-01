package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {

  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;

  /**
   * The pose estimates from the limelights in the following order {shooterLimelight, frontLeftLimelight, frontRightLimelight}
   */
  private PoseEstimate[] limelightEstimates = {new PoseEstimate(), new PoseEstimate(), new PoseEstimate()};

  public VisionSubsystem() {
    visionThread(VisionConstants.SHOOTER_LIMELIGHT_NUMBER);
    visionThread(VisionConstants.FRONT_LEFT_LIMELIGHT_NUMBER);
    visionThread(VisionConstants.FRONT_RIGHT_LIMELIGHT_NUMBER);
  }

  /**
   * Checks if the specified limelight can fully see one or more April Tag.
   * @param limelightNumber the number of the limelight
   * @return true if the limelight can fully see one or more April Tag
   */
  public boolean canSeeAprilTags(int limelightNumber) {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // Different Limelights have different FOVs
    if (getLimelightName(limelightNumber).equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
      return getNumberOfAprilTags(limelightNumber) != 0
        && Math.abs(LimelightHelpers.getTX(getLimelightName(limelightNumber))) <= VisionConstants.LL3G_FOV_MARGIN_OF_ERROR;
    }
    return getNumberOfAprilTags(limelightNumber) != 0
      && Math.abs(LimelightHelpers.getTX(getLimelightName(limelightNumber))) <= VisionConstants.LL3_FOV_MARGIN_OF_ERROR;
  }

  /**
   * Gets the JSON dump from the specified limelight and puts it into a PoseEstimate object,
   * which is then placed into its corresponding spot in the limelightEstimates array.
   * @param limelightNumber the number of the limelight
   */
  public void updateLimelightPoseEstimate(int limelightNumber) {
    if (canSeeAprilTags(limelightNumber)) {
      // MegaTag2 is much more accurate, but only use it when the robot isn't rotating too fast
      if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
        LimelightHelpers.SetRobotOrientation(VisionConstants.SHOOTER_LIMELIGHT_NAME, headingDegrees, 0, 0, 0, 0, 0);
        limelightEstimates[limelightNumber] = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName(limelightNumber));
      }
      limelightEstimates[limelightNumber] = LimelightHelpers.getBotPoseEstimate_wpiBlue(getLimelightName(limelightNumber));
    } else {
      limelightEstimates[limelightNumber] = new PoseEstimate();
    }
  }

  /**
   * Gets the pose of the robot calculated by specified limelight via any April
   * Tags it sees
   * @param limelightNumber the number of the limelight
   * @return the pose of the robot, if the limelight can't see any April Tags,
   * it will return 0 for x, y, and theta
   */
  public Pose2d getPoseFromAprilTags(int limelightNumber) {
    return limelightEstimates[limelightNumber].pose;
  }

  /**
   * Returns how many april tags the limelight that is being used for pose
   * estimation can see.
   */
  public int getNumberOfAprilTags(int limelightNumber) {
    return limelightEstimates[limelightNumber].tagCount;
  }

  /**
   * Returns the timestamp for the vision measurement from the limelight that
   * is being used for pose estimation.
   */
  public double getTimeStampSeconds(int limelightNumber) {
    return limelightEstimates[limelightNumber].timestampSeconds / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being
   * used for pose estimation calculated the robot's pose. It adds the
   * pipeline latency, capture latency, and json parsing latency.
   */
  public double getLatencySeconds(int limelightNumber) {
    // TODO: Verify this works
    return (limelightEstimates[limelightNumber].latency) / 1000.0;
  }

  /**
   * Gets the average distance between the specified limelight and the April Tags it sees
   * @param limelightNumber the number of the limelight
   * @return the average distance between the robot and the April Tag(s) in meters
   */
  public double getLimelightAprilTagDistance(int limelightNumber) {
    if (canSeeAprilTags(limelightNumber)) {
      // TODO: Verify this is in meters (there's almost no way its not, but just make sure)
      return limelightEstimates[limelightNumber].avgTagDist; // or RawFiducial.disToCamera??
    }
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  /**
   * Sets the heading and heading rate of the robot, this is used
   * for deciding between MegaTag 1 and 2 for pose estimation.
   * @param headingDegrees the angle the robot is facing in degrees (0 degrees facing the red alliance)
   * @param headingRateDegrees the rate the robot is rotating, CCW positive
   */
  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    this.headingDegrees = headingDegrees;
    this.headingRateDegreesPerSecond = headingRateDegrees;
  }

  /**
   * 0 = Shooter
   * 1 = Front Left
   * 2 = Front Right
   * @param limelightNumber the limelight
   * @return
   */
  public String getLimelightName(int limelightNumber) {
    if (limelightNumber == 0) {
      return VisionConstants.SHOOTER_LIMELIGHT_NAME;
    } else if (limelightNumber == 1) {
      return VisionConstants.FRONT_LEFT_LIMELIGHT_NAME;
    } else if (limelightNumber == 2) {
      return VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME;
    }
     throw new IllegalArgumentException();
  }

  /**
   * Gets the pose calculated the last time a limelight saw an April Tag
   */
  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  public void visionThread(int limelightNumber) {
    try {
      new Thread(() -> {
        while (true) {
          updateLimelightPoseEstimate(limelightNumber);
          // This is to keep track of the last valid pose calculated by the limelights
          // it is used when the driver resets the robot odometry to the limelight calculated position
          if (canSeeAprilTags(limelightNumber)) {
            lastSeenPose = getPoseFromAprilTags(limelightNumber);
          }
        }
      }).start();
    } catch (Exception e) {}
  } 

  @Override
  public void periodic() {}

}
