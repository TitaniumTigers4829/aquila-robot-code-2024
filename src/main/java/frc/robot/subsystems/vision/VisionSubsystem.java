package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {

  private LimelightResults currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.SHOOTER_LIMELIGHT_NAME);
  private String currentlyUsedLimelight = VisionConstants.SHOOTER_LIMELIGHT_NAME;
  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegrees = 0;

  public VisionSubsystem() {}

  /**
   * Returns true if the limelight(s) can fully see one or more April Tag.
   */
  public boolean canSeeAprilTags() {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // Different Limelights have different FOVs
    if (currentlyUsedLimelight.equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
          return LimelightHelpers.getFiducialID(currentlyUsedLimelight) != -1
      && Math.abs(LimelightHelpers.getTX(currentlyUsedLimelight)) <= VisionConstants.LL3G_FOV_MARGIN_OF_ERROR;
    }
    return LimelightHelpers.getFiducialID(currentlyUsedLimelight) != -1
      && Math.abs(LimelightHelpers.getTX(currentlyUsedLimelight)) <= VisionConstants.LL3_FOV_MARGIN_OF_ERROR;
  }

  /**
   * Returns the pose of the robot calculated by the limelight. If there
   * are multiple limelights that can see april tags, it uses the limelight
   * that is closest to an april tag. 
   */
  public Pose2d getPoseFromAprilTags() {
    if (canSeeAprilTags()) {
      // MegaTag2 is much more accurate, but only use it when the robot isn't rotating too fast
      if (headingRateDegrees < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
        LimelightHelpers.SetRobotOrientation(VisionConstants.SHOOTER_LIMELIGHT_NAME, headingDegrees, 0, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(currentlyUsedLimelight).pose;
      }
      
      Pose2d botPose = LimelightHelpers.getBotPose2d(currentlyUsedLimelight);
      // The origin of botpose is at the center of the field
      double robotX = botPose.getX() + FieldConstants.FIELD_LENGTH_METERS / 2.0;
      double robotY = botPose.getY() + FieldConstants.FIELD_WIDTH_METERS / 2.0;
      Rotation2d robotRotation = botPose.getRotation();
      lastSeenPose = new Pose2d(robotX, robotY, robotRotation);
      return new Pose2d(robotX, robotY, robotRotation);
    } else {
      return new Pose2d();
    }
  }

  /**
   * Returns the distance in meters from the limelight(s) to the closest 
   * april tag that they can see.
   */
  public double getDistanceFromClosestAprilTag() {
    if (canSeeAprilTags()) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(currentlyUsedLimelight);
      double distance = getLimelightAprilTagDistance(closestAprilTagID);
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
    return currentlyUsedLimelightResults.targetingResults.targets_Fiducials.length;
  }

  /**
   * Returns the timestamp for the vision measurement from the limelight that
   * is being used for pose estimation.
   */
  public double getTimeStampSeconds() {
    return currentlyUsedLimelightResults.targetingResults.timestamp_LIMELIGHT_publish / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being
   * used for pose estimation calculated the robot's pose. It adds the
   * pipeline latency, capture latency, and json parsing latency.
   */
  public double getLatencySeconds() {
    return (currentlyUsedLimelightResults.targetingResults.latency_capture 
    + currentlyUsedLimelightResults.targetingResults.latency_pipeline 
    + currentlyUsedLimelightResults.targetingResults.latency_jsonParse) / 1000.0;
  }

  /**
   * Calculates the distance between the specified robot and april tag.
   * This method should only be called once there has been a check for if
   * the limelights can see april tags.
   */
  private double getLimelightAprilTagDistance(int aprilTagID) {
    if (aprilTagID >= 1 && aprilTagID <= 16) {
      double aprilTagX = VisionConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][0]; // April tag id starts at 1
      double aprilTagY = VisionConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][1];
      Pose2d pose = getPoseFromAprilTags();
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

  public String getCurrentlyUsedLimelightName() {
    return currentlyUsedLimelight;
  }



  @Override
  public void periodic() {
    // Every periodic chooses the limelight to use based off of their distance from april tags
    // This code has the limelights alternating in updating their results every other loop.
    // It makes sense because they run at ~12hz, where the roborio runs at 50hz.
    if (currentlyUsedLimelight.equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
      currentlyUsedLimelight = VisionConstants.FRONT_LEFT_LIMELIGHT_NAME;
    } else if (currentlyUsedLimelight.equals(VisionConstants.FRONT_LEFT_LIMELIGHT_NAME)) {
      currentlyUsedLimelight = VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME;
    } else if (currentlyUsedLimelight.equals(VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME)) {
      currentlyUsedLimelight = VisionConstants.SHOOTER_LIMELIGHT_NAME;
    }


    try{
      new Thread(() -> {
        while(true){
         // Gets the JSON dump from the currently used limelight
        currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(currentlyUsedLimelight);
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
          }
        }
      }).start();
    }catch(Exception e){}
    
  }

}