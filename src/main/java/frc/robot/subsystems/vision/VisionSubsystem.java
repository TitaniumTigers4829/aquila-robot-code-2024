package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.extras.LimelightHelpers;
// import frc.robot.extras.SmartDashboardLogger;
import frc.robot.extras.LimelightHelpers.LimelightResults;
import frc.robot.extras.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.extras.SingleLinearInterpolator;

public class VisionSubsystem extends SubsystemBase {

  private LimelightResults currentlyUsedLimelightResults;
  private LimelightResults frontLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.FRONT_LIMELIGHT_NAME);;
  private LimelightResults backLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.BACK_LIMELIGHT_NAME);;
  private String currentlyUsedLimelight = VisionConstants.FRONT_LIMELIGHT_NAME;
  private boolean wasFrontLimelightUsedLast = false;
  
  SingleLinearInterpolator leftSpeakerLookupValues;
  SingleLinearInterpolator rightSpeakerLookupValues;
  SingleLinearInterpolator speakerAngleLookupValues;

  SingleLinearInterpolator leftAmpLookupValues;
  SingleLinearInterpolator rightAmpLookupValues;
  SingleLinearInterpolator ampAngleLookupValues;

  public VisionSubsystem() {
    currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(VisionConstants.FRONT_LIMELIGHT_NAME);
    
    leftSpeakerLookupValues = new SingleLinearInterpolator(ShooterConstants.LEFT_MOTOR_SPEAKER_VALUES);
    rightSpeakerLookupValues = new SingleLinearInterpolator(ShooterConstants.RIGHT_MOTOR_SPEAKER_VALUES);
    speakerAngleLookupValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_PIVOT_SPEAKER_POSITION);

    leftAmpLookupValues = new SingleLinearInterpolator(ShooterConstants.LEFT_MOTOR_AMP_VALUES);
    rightAmpLookupValues = new SingleLinearInterpolator(ShooterConstants.RIGHT_MOTOR_AMP_VALUES);
    ampAngleLookupValues = new SingleLinearInterpolator(ShooterConstants.SPEAKER_PIVOT_AMP_POSITION);
  }

  public boolean canSeeAprilTags() {
    return LimelightHelpers.getFiducialID(currentlyUsedLimelight) != -1;
  }


  public Pose2d getPoseFromAprilTags() {
    Pose2d botPose = LimelightHelpers.getBotPose2d(currentlyUsedLimelight);
    // The origin of botpose is at the center of the field
    double robotX = botPose.getX() + FieldConstants.FIELD_LENGTH_METERS / 2;
    double robotY = botPose.getY() + FieldConstants.FIELD_WIDTH_METERS / 2;
    Rotation2d robotRotation = botPose.getRotation();
    return new Pose2d(robotX, robotY, robotRotation);
  }

  public double getDistanceFromClosestAprilTag() {
    if (canSeeAprilTags()) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(currentlyUsedLimelight);
      return getLimelightAprilTagDistance(closestAprilTagID);
    }
    
    // To be safe returns a big distance from the april tags
    return Double.MAX_VALUE;
  }

  public int getNumberOfAprilTags() {
    return currentlyUsedLimelightResults.targetingResults.targets_Fiducials.length;
  }

  public double getTimeStampSeconds() {
    return currentlyUsedLimelightResults.targetingResults.timestamp_LIMELIGHT_publish / 1000.0;
  }

  public double getLatencySeconds() {
    return (currentlyUsedLimelightResults.targetingResults.latency_capture 
    + currentlyUsedLimelightResults.targetingResults.latency_pipeline 
    + currentlyUsedLimelightResults.targetingResults.latency_jsonParse) / 1000.0;
  }


  public void cropLimelights(double[][] cropValues) {
    LimelightHelpers.setCropWindow(
      VisionConstants.FRONT_LIMELIGHT_NAME, 
      cropValues[0][0],
      cropValues[0][1],
      cropValues[0][2],
      cropValues[0][3]
    );
  }

  // public void setLimelightsPipeline(LimelightsPipelines limelightPipeline) {
  //   LimelightHelpers.setPipelineIndex(VisionConstants.FRONT_LIMELIGHT_NAME, limelightPipeline.getID());
  // }

  /**
   * Calculates the distance between the specified robot and april tag.
   * This method should only be called once there has been a check for if
   * the limelights can see april tags.
   */
  private double getLimelightAprilTagDistance(int aprilTagID) {
    if (aprilTagID >= 1) {
      double aprilTagX = VisionConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][0]; // April tag id starts at 1
      double aprilTagY = VisionConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][1];
      // Added a little optimization
      Pose2d pose = getPoseFromAprilTags();
      double robotX = pose.getX();
      double robotY = pose.getY();
      return Math.sqrt(Math.pow(aprilTagX - robotX, 2) + Math.pow(aprilTagY - robotY, 2));
    }

    // To be safe returns a big distance from the april tags
    return Double.MAX_VALUE;
  }

  @Override
  public void periodic() {
    // Every periodic chooses the limelight to use based off of their distance from april tags
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
    // SmartDashboardLogger.infoString("Limelight Pos", getPoseFromAprilTags().toString());

    // Flashes the limelight LEDs if they can't see an april tag
    if (!canSeeAprilTags()) {
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.FRONT_LIMELIGHT_NAME);
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.BACK_LIMELIGHT_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(VisionConstants.FRONT_LIMELIGHT_NAME);
      LimelightHelpers.setLEDMode_ForceOff(VisionConstants.BACK_LIMELIGHT_NAME);
    }
  }

  public double[] getSpeakerStuff() {
    double[] to_return = new double[3];
    Translation2d pose = getPoseFromAprilTags().getTranslation();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed;
    if (alliance.isPresent()) {
      isRed = alliance.get() == Alliance.Red;
    } else {
      isRed = true;
    }
    Translation2d other = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);

    double distance = pose.getDistance(other);
    to_return[0] = leftSpeakerLookupValues.getLookupValue(distance);
    to_return[1] = rightSpeakerLookupValues.getLookupValue(distance);
    to_return[2] = speakerAngleLookupValues.getLookupValue(distance);
    return to_return;
  }

  public double[] getAmpStuff() {
    double[] to_return = new double[3];
    Translation2d pose = getPoseFromAprilTags().getTranslation();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed;
    if (alliance.isPresent()) {
      isRed = alliance.get() == Alliance.Red;
    } else {
      isRed = true;
    }
    Translation2d other = isRed ? new Translation2d(FieldConstants.RED_AMP_X, FieldConstants.RED_AMP_Y) : new Translation2d(FieldConstants.BLUE_AMP_X, FieldConstants.BLUE_AMP_Y);
    double distance = pose.getDistance(other);
    to_return[0] = leftAmpLookupValues.getLookupValue(distance);
    to_return[1] = rightAmpLookupValues.getLookupValue(distance);
    to_return[2] = ampAngleLookupValues.getLookupValue(distance);
    return to_return;
  }

}