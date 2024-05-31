package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.extras.LimelightHelpers.LimelightResults;
import frc.robot.extras.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {

  private PoseEstimate Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.SHOOTER_LIMELIGHT_NAME);
  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegrees = 0;

  private PoseEstimate[] limelightEstimates = {new PoseEstimate(), new PoseEstimate(), new PoseEstimate()};

  public VisionSubsystem() {
    visionThread(0);
    visionThread(1);
    visionThread(2);
  }

  /**
   * Returns true if the limelight(s) can fully see one or more April Tag.
   */
  public boolean canSeeAprilTags(int index) {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // Different Limelights have different FOVs
    if (getLimelightName(index).equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
      
          return limelightEstimates[index].rawFiducials.length != 0
      && Math.abs(LimelightHelpers.getTX(getLimelightName(index))) <= VisionConstants.LL3G_FOV_MARGIN_OF_ERROR;
    }
    return  limelightEstimates[index].rawFiducials.length != 0
      && Math.abs(LimelightHelpers.getTX(getLimelightName(index))) <= VisionConstants.LL3_FOV_MARGIN_OF_ERROR;
  }

  /**
   * Returns the pose of the robot calculated by the limelight. If there
   * are multiple limelights that can see april tags, it uses the limelight
   * that is closest to an april tag.
   */
  public void getPoseFromAprilTags(int index) {
    if (canSeeAprilTags(index)) {
      // MegaTag2 is much more accurate, but only use it when the robot isn't rotating too fast
      if (headingRateDegrees < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
        LimelightHelpers.SetRobotOrientation(VisionConstants.SHOOTER_LIMELIGHT_NAME, headingDegrees, 0, 0, 0, 0, 0);
        limelightEstimates[index] = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName(index));
      }

      // LimelightHelpers.getJSONDump();
      
      return limelightEstimates[index].pose;
      // // The origin of botpose is at the center of the field
      // double robotX = botPose.getX() + FieldConstants.FIELD_LENGTH_METERS / 2.0;
      // double robotY = botPose.getY() + FieldConstants.FIELD_WIDTH_METERS / 2.0;
      // Rotation2d robotRotation = botPose.getRotation();
      // lastSeenPose = new Pose2d(robotX, robotY, robotRotation);
      // return new Pose2d(robotX, robotY, robotRotation);
    } else {
      return new Pose2d();
    }
  }

  /**
   * Returns the distance in meters from the limelight(s) to the closest 
   * april tag that they can see.
   */
  public double getDistanceFromClosestAprilTag(int index) {
    if (canSeeAprilTags(, index)) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(getLimelightName(index));
      double distance = getLimelightAprilTagDistance(closestAprilTagID, index);
      // SmartDashboard.putNumber("distance from apriltag", distance);
      return distance;
    }
    
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  /**
   * Returns how many april tags the limelight that is being used for pose
   * estimation can see.
   */
  public int getNumberOfAprilTags() {
    return Estimate.tagCount;
  }

  /**
   * Returns the timestamp for the vision measurement from the limelight that
   * is being used for pose estimation.
   */
  public double getTimeStampSeconds() {
    return Estimate.timestampSeconds / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being
   * used for pose estimation calculated the robot's pose. It adds the
   * pipeline latency, capture latency, and json parsing latency.
   */
  public double getLatencySeconds() {
    return (Estimate.latency) / 1000.0;
  }

  /**
   * Calculates the distance between the specified robot and april tag.
   * This method should only be called once there has been a check for if
   * the limelights can see april tags.
   */
  private double getLimelightAprilTagDistance(int aprilTagID, int index) {
    if (aprilTagID >= 1 && aprilTagID <= 16) {
      double aprilTagX = VisionConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][0]; // April tag id starts at 1
      double aprilTagY = VisionConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][1];
      Pose2d pose = getPoseFromAprilTags(index);
      double robotX = pose.getX();
      double robotY = pose.getY();
      // Uses distance formula
      return Math.sqrt(Math.pow(aprilTagX - robotX, 2) + Math.pow(aprilTagY - robotY, 2));
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
    this.headingRateDegrees = headingRateDegrees;
  }

  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  private void setLimelightPose(int index, PoseEstimate pose) {
    limelightEstimates[index] = pose;
  }

  public PoseEstimate[] getLimelightPoses() {
    return limelightEstimates;
  }

  /**
   * 0 = Shooter
   * 1 = Front Left
   * 2 = Front Right
   * @param num
   * @return
   */
  public String getLimelightName(int num) {
    if (num == 0) {
      return VisionConstants.SHOOTER_LIMELIGHT_NAME;
    } else if (num == 1) {
      return VisionConstants.FRONT_LEFT_LIMELIGHT_NAME;
    } else if (num == 2) {
      return VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME;
    }
     throw new IllegalArgumentException();
  }

  public void visionThread(int index) {
    try {
      new Thread(() -> {
        while (true) {
        setLimelightPose(index, getPoseFromAprilTags(index));
        }
      }).start();
    } catch (Exception e) {}
  } 

  @Override
  public void periodic() {
    
  }

}
