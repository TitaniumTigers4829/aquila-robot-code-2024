package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.PoseEstimate;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class VisionSubsystem extends SubsystemBase {

  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;
  private Lock poseLock = new ReentrantLock();
  // private AtomicBoolean running = new AtomicBoolean(true);

  /**
   * The pose estimates from the limelights in the following order {shooterLimelight,
   * frontLeftLimelight, frontRightLimelight}
   */
  private PoseEstimate[] limelightEstimates = {
    new PoseEstimate(), new PoseEstimate(), new PoseEstimate()
  };

  public VisionSubsystem() {}

  /**
   * Checks if the specified limelight can fully see one or more April Tag.
   *
   * @param limelightNumber the number of the limelight
   * @return true if the limelight can fully see one or more April Tag
   */
  public boolean canSeeAprilTags(int limelightNumber) {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // Different Limelights have different FOVs
    if (getNumberOfAprilTags(limelightNumber) > 0
        && getNumberOfAprilTags(limelightNumber) <= VisionConstants.APRIL_TAG_POSITIONS.length) {
      if (getLimelightName(limelightNumber).equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
        return Math.abs(LimelightHelpers.getTX(getLimelightName(limelightNumber)))
            <= VisionConstants.LL3G_FOV_MARGIN_OF_ERROR;
      } else {
        return Math.abs(LimelightHelpers.getTX(getLimelightName(limelightNumber)))
            <= VisionConstants.LL3_FOV_MARGIN_OF_ERROR;
      }
    }
    return false;
  }

  /**
   * Gets the JSON dump from the specified limelight and puts it into a PoseEstimate object, which
   * is then placed into its corresponding spot in the limelightEstimates array.
   *
   * @param limelightNumber the number of the limelight
   */
  public void updateLimelightPoseEstimate(int limelightNumber) {
    if (!canSeeAprilTags(limelightNumber)) {
      limelightEstimates[limelightNumber] = new PoseEstimate();
    }

    if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
      LimelightHelpers.SetRobotOrientation(
          getLimelightName(limelightNumber), headingDegrees, 0, 0, 0, 0, 0);
      limelightEstimates[limelightNumber] = getMegaTag2PoseEstimate(limelightNumber);
    } else {
      limelightEstimates[limelightNumber] = getMegaTag1PoseEstimate(limelightNumber);
    }
  }

  /**
   * Gets the MegaTag1 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelightNumber the number of the limelight
   * @return the MegaTag1 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag1PoseEstimate(int limelightNumber) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(getLimelightName(limelightNumber));
  }

  /**
   * Gets the MegaTag2 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelightNumber the number of the limelight
   * @return the MegaTag2 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag2PoseEstimate(int limelightNumber) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName(limelightNumber));
  }

  /**
   * Gets the pose of the robot calculated by specified limelight via any April Tags it sees
   *
   * @param limelightNumber the number of the limelight
   * @return the pose of the robot, if the limelight can't see any April Tags, it will return 0 for
   *     x, y, and theta
   */
  public Pose2d getPoseFromAprilTags(int limelightNumber) {
    return limelightEstimates[limelightNumber].pose;
  }

  /** Returns how many april tags the limelight that is being used for pose estimation can see. */
  public int getNumberOfAprilTags(int limelightNumber) {
    return limelightEstimates[limelightNumber].tagCount;
  }

  /**
   * Returns the timestamp for the vision measurement from the limelight that is being used for pose
   * estimation.
   */
  public double getTimeStampSeconds(int limelightNumber) {
    return limelightEstimates[limelightNumber].timestampSeconds / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being used for pose estimation
   * calculated the robot's pose. It adds the pipeline latency, capture latency, and json parsing
   * latency.
   */
  public double getLatencySeconds(int limelightNumber) {
    return (limelightEstimates[limelightNumber].latency) / 1000.0;
  }

  /**
   * Gets the average distance between the specified limelight and the April Tags it sees
   *
   * @param limelightNumber the number of the limelight
   * @return the average distance between the robot and the April Tag(s) in meters
   */
  public double getLimelightAprilTagDistance(int limelightNumber) {
    if (canSeeAprilTags(limelightNumber)) {
      return limelightEstimates[limelightNumber].avgTagDist;
    }
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  /**
   * Sets the heading and heading rate of the robot, this is used for deciding between MegaTag 1 and
   * 2 for pose estimation.
   *
   * @param headingDegrees the angle the robot is facing in degrees (0 degrees facing the red
   *     alliance)
   * @param headingRateDegrees the rate the robot is rotating, CCW positive
   */
  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    this.headingDegrees = headingDegrees;
    this.headingRateDegreesPerSecond = headingRateDegrees;
  }

  /**
   * Gets the limelight name associated with the specified limelight number/index
   *
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
    throw new IllegalArgumentException("You entered a number for a non-existent limelight");
  }

  /** Gets the pose calculated the last time a limelight saw an April Tag */
  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelightNumber the limelight number
   */
  private void checkAndUpdatePoseOnce(int limelightNumber) {
    double last_TX = 0;
    double last_TY = 0;
    poseLock.lock();
    try {
    double current_TX = LimelightHelpers.getTX(getLimelightName(limelightNumber));
    double current_TY = LimelightHelpers.getTY(getLimelightName(limelightNumber));

    if (current_TX != last_TX || current_TY != last_TY) {
      updateLimelightPoseEstimate(limelightNumber);
      // running.set(true);
      if (!canSeeAprilTags(limelightNumber)) {
        lastSeenPose = getMegaTag1PoseEstimate(limelightNumber).pose;
      }
    } else {
      limelightEstimates[limelightNumber] = new PoseEstimate();
      // stop();
    }

    last_TX = current_TX;
    last_TY = current_TY;
    } finally {
    poseLock.unlock();
  }
  }

  /**
   * Starts a thread which performs a one-time check for updates in pose for the specified
   * limelight.
   *
   * @param limelightNumber the limelight number
   */
  public void visionThread(int limelightNumber) {
    new Thread(
            () -> {
              checkAndUpdatePoseOnce(limelightNumber);
            })
        .start();
  }

  /**
   * runs the vision thread
   *
   * @param limelightNumber
   */
  // public void runThread(int limelightNumber) {
  //   while (running.get()) {
  //     visionThread(limelightNumber);
  //   }
  // }

  // /** sets running to false */
  // public void stop() {
  //   running.set(false);
  // }

  // Override periodic method to start the vision threads at the beginning of each subsystem tick
  @Override
  public void periodic() {
    visionThread(VisionConstants.SHOOTER_LIMELIGHT_NUMBER);
    visionThread(VisionConstants.FRONT_LEFT_LIMELIGHT_NUMBER);
    visionThread(VisionConstants.FRONT_RIGHT_LIMELIGHT_NUMBER);
  }
}
