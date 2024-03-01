package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.LimelightResults;
import frc.robot.extras.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {

  private LimelightResults currentlyUsedLimelightResults;
  private LimelightResults frontLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.FRONT_LIMELIGHT_NAME);
  private LimelightResults backLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.BACK_LIMELIGHT_NAME);
  private String currentlyUsedLimelight = VisionConstants.FRONT_LIMELIGHT_NAME;
  private boolean wasFrontLimelightUsedLast = false;
  
  // SingleLinearInterpolator leftSpeakerLookupValues;
  // SingleLinearInterpolator speakerAngleLookupValues;

  // SingleLinearInterpolator leftAmpLookupValues;
  // SingleLinearInterpolator ampAngleLookupValues;

  public VisionSubsystem() {
    currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.FRONT_LIMELIGHT_NAME);
    
    // leftSpeakerLookupValues = new SingleLinearInterpolator(ShooterConstants.LEFT_MOTOR_SPEAKER_VALUES);
    // speakerAngleLookupValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_PIVOT_SPEAKER_POSITION);

    // leftAmpLookupValues = new SingleLinearInterpolator(ShooterConstants.LEFT_MOTOR_AMP_VALUES);
    // ampAngleLookupValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_PIVOT_AMP_POSITION);
  }

  /**
   * Returns true if the limelight(s) can fully see one or more April Tag.
   */
  public boolean canSeeAprilTags() {
    // First checks if it can see an april tag, then checks if it is fully in frame
    return LimelightHelpers.getFiducialID(currentlyUsedLimelight) != -1
      && Math.abs(LimelightHelpers.getTX(currentlyUsedLimelight)) <= VisionConstants.FOV_MARGIN_OF_ERROR;
  }


  /**
   * Returns the pose of the robot calculated by the limelight. If there
   * are multiple limelights that can see april tags, it uses the limelight
   * that is closest to an april tag. 
   */
  public Pose2d getPoseFromAprilTags() {
    Pose2d botPose = LimelightHelpers.getBotPose2d(currentlyUsedLimelight);
    // The origin of botpose is at the center of the field
    double robotX = botPose.getX() + FieldConstants.FIELD_LENGTH_METERS / 2;
    double robotY = botPose.getY() + FieldConstants.FIELD_WIDTH_METERS / 2;
    Rotation2d robotRotation = botPose.getRotation();
    return new Pose2d(robotX, robotY, robotRotation);
  }

  /**
   * Returns the distance in meters from the limelight(s) to the closest 
   * april tag that they can see.
   */
  public double getDistanceFromClosestAprilTag() {
    if (canSeeAprilTags()) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(currentlyUsedLimelight);
      return getLimelightAprilTagDistance(closestAprilTagID);
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
   * Returns how many april tags the limelight that is being used for pose
   * estimation can see.
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
    if (aprilTagID >= 1) {
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

  @Override
  public void periodic() {
    // Every periodic chooses the limelight to use based off of their distance from april tags
    // This code has the limelights alternating in updating their results every other loop.
    // It makes sense because they run at ~12hz, where the roborio runs at 50hz.
    if (wasFrontLimelightUsedLast) {
      backLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.BACK_LIMELIGHT_NAME);
    } else {
      frontLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.FRONT_LIMELIGHT_NAME);
    }

    wasFrontLimelightUsedLast = !wasFrontLimelightUsedLast;

    LimelightTarget_Fiducial[] frontLimelightAprilTags = frontLimelightResults.targetingResults.targets_Fiducials;
    LimelightTarget_Fiducial[] backLimelightAprilTags = backLimelightResults.targetingResults.targets_Fiducials;

    // Gets the distance from the closest april tag. If it can't see one, returns a really big number.
    double frontLimelightDistance = frontLimelightAprilTags.length > 0
      ? getLimelightAprilTagDistance((int) frontLimelightAprilTags[0].fiducialID) : Double.MAX_VALUE;
    double backLimelightDistance = backLimelightAprilTags.length > 0
      ? getLimelightAprilTagDistance((int) backLimelightAprilTags[0].fiducialID) : Double.MAX_VALUE;

    currentlyUsedLimelight = frontLimelightDistance <= backLimelightDistance 
      ? VisionConstants.FRONT_LIMELIGHT_NAME : VisionConstants.BACK_LIMELIGHT_NAME;
    currentlyUsedLimelightResults = currentlyUsedLimelight == VisionConstants.FRONT_LIMELIGHT_NAME
      ? frontLimelightResults : backLimelightResults;

    SmartDashboard.putNumber("distance from apriltag", getDistanceFromClosestAprilTag());

    // Turns the limelight LEDs on if they can't see an april tag
    if (!canSeeAprilTags()) {
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.FRONT_LIMELIGHT_NAME);
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.BACK_LIMELIGHT_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(VisionConstants.FRONT_LIMELIGHT_NAME);
      LimelightHelpers.setLEDMode_ForceOff(VisionConstants.BACK_LIMELIGHT_NAME);
    }
  }


  /**
   * Gets the data needed for shooting into the speaker
   * @return double[leftFlywheelSpeed shooterAngle, desiredHeading]
   */
  // public double[] getSpeakerShooterData() {
  //   double[] shooterData = new double[3];
  //   Translation2d pose = getPoseFromAprilTags().getTranslation();
  //   Optional<Alliance> alliance = DriverStation.getAlliance();
  //   boolean isRed;
  //   if (alliance.isPresent()) {
  //     isRed = alliance.get() == Alliance.Red;
  //   } else {
  //     isRed = true;
  //   }
  //   Translation2d speakerPosition = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);

  //   double distance = pose.getDistance(speakerPosition);
  //   shooterData[0] = leftSpeakerLookupValues.getLookupValue(distance); //Use a linear interpolator to determine speed for shooter
  //   shooterData[1] = speakerAngleLookupValues.getLookupValue(distance); //Use a linear interpolator to determine angle for shooter
  //   shooterData[2] = Math.atan2(speakerPosition.getY() - pose.getY(), speakerPosition.getX() - pose.getX()) * 180.0 * Math.PI; //Determine desired rotation of robot for shooter
  //   return shooterData;
  // }
}