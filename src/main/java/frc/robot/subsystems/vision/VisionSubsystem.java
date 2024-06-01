package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
   * Gets the limelight name associated with the specified limelight number/index
   * @param limelightNumber the limelight number
   * @return 0 = limelight-shooter, 1 = limelight-left, 2 = limelight-right
   */
  public String getLimelightName(int limelightNumber) {
    if (limelightNumber == 0) {
      return VisionConstants.SHOOTER_LIMELIGHT_NAME;
    } else if (limelightNumber == 1) {
      return VisionConstants.FRONT_LEFT_LIMELIGHT_NAME;
    } else if (limelightNumber == 2) {
      return VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME;
    }
     throw new IllegalArgumentException("You enterd a number for a non-existent limelight");
  }

  /**
   * Gets the pose calculated the last time a limelight saw an April Tag
   */
  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  /**
   * This makes a thread which updates the pose estimate for the specified limelight.
   * Because we have 3 limelights, if we are trying to parse the JSON dump  for each limelight
   * one after another every scheduler loop, we will have loop overruns, so we opted to have them 
   * run in parallel like this.
   * @param limelightNumber the limelight number
   */
  public void visionThread(int limelightNumber) {
    try {
      new Thread(() -> {
        double last_TX = 0;
        double last_TY = 0;

        while (true) {
          double current_TX = LimelightHelpers.getTX(getLimelightName(limelightNumber));
          double current_TY = LimelightHelpers.getTY(getLimelightName(limelightNumber));

          // This checks if the limelight reading is new. The reasoning being that if the TX and TY
          // are EXACTLY the same, it hasn't updated yet with a new reading. We are doing it this way,
          // because to get the timestamp of the reading, you need to parse the JSON dump which can be
          // very demanding whereas this only has to get the Network Table entries for TX and TY.
          if (last_TX != current_TX && last_TY != current_TY) {
            updateLimelightPoseEstimate(limelightNumber);
            // This is to keep track of the last valid pose calculated by the limelights
            // it is used when the driver resets the robot odometry to the limelight calculated position
            if (canSeeAprilTags(limelightNumber)) {
              lastSeenPose = getPoseFromAprilTags(limelightNumber);
            }
          }

          last_TX = current_TX;
          last_TY = current_TY;
        }
      }).start();
    } catch (Exception e) {
      e.printStackTrace();
    }
  } 

  @Override
  public void periodic() {}

}
