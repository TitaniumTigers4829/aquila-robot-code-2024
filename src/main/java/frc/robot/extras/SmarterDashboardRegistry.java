// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extras;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class SmarterDashboardRegistry {
  private static double[] pose = new double[]{0, 0, 0};
  private static double[] limelight_pose = new double[]{0, 0, 0};
  private static double[] orientation = new double[]{0, 0, 0};
  // isActive, isRedAlliance, isSpeaker
  private static boolean[] pathData = new boolean[]{false, false, false};

  private static boolean isRed;
  private static Pose2d ampPos;
  private static Translation2d speakerPos;
  private static Translation2d loadingStationPos;
  private static Translation2d passingPos;

  private static double amplifiedTimeLeft = 10.0;
  private static boolean isAmplified;
  private static Timer timer;

  public static void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRed = alliance.get() == Alliance.Red;
    } else {
      isRed = true;
    }

    ampPos = isRed ? new Pose2d(FieldConstants.RED_AMP_X, FieldConstants.RED_AMP_Y, FieldConstants.RED_AMP_ROTATION) : new Pose2d(FieldConstants.BLUE_AMP_X, FieldConstants.BLUE_AMP_Y, FieldConstants.BLUE_AMP_ROTATION);
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    loadingStationPos = isRed ? new Translation2d(FieldConstants.RED_LOADING_STATION_X, FieldConstants.RED_LOADING_STATION_Y) : new Translation2d(FieldConstants.BLUE_LOADING_STATION_X, FieldConstants.BLUE_LOADING_STATION_Y);
    passingPos = isRed ? new Translation2d(FieldConstants.RED_PASSING_X, FieldConstants.RED_PASSING_Y) : new Translation2d(FieldConstants.BLUE_PASSING_X, FieldConstants.BLUE_PASSING_Y);
  }

  public static void setPose(Pose2d robotPose) {
    pose[0] = robotPose.getX();
    pose[1] = robotPose.getY();
    pose[2] = robotPose.getRotation().getRadians();
    SmartDashboard.putNumberArray("botPose", pose);
  }

  public static boolean isRed() {
    return isRed;
  }

  public static void setLimelightPose(Pose2d limelightPose) {
    limelight_pose[0] = limelightPose.getX();
    limelight_pose[1] = limelightPose.getY();
    limelight_pose[2] = limelightPose.getRotation().getDegrees();
    SmartDashboard.putNumberArray("limelightPose", limelight_pose);
  }

  public static void setOrientation(Rotation3d robotOrientation) {
    orientation[0] = robotOrientation.getX();
    orientation[1] = robotOrientation.getY();
    orientation[2] = robotOrientation.getZ();
    SmartDashboard.putNumberArray("botOrientation", orientation);
  }

  public static void updatePathData(boolean isActive, boolean isRed, boolean isSpeaker) {
    pathData[0] = isActive;
    pathData[1] = isRed;
    pathData[2] = isSpeaker;
    SmartDashboard.putBooleanArray("pathData", pathData);
  }

  public static Pose2d getPose() {
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[2]));
  }

  public static Pose2d getLimelightPose() {
    return new Pose2d(limelight_pose[0], limelight_pose[1], Rotation2d.fromDegrees(limelight_pose[2]));
  }

  public static boolean[] getPathData() {
    return pathData;
  }

  public static Translation2d getSpeakerPos() {
    return speakerPos;
  }

  public static Pose2d getAmpPos() {
    return ampPos;
  }

  public static Translation2d getPassingPos() {
    return passingPos;
  }

  public static Translation2d getLoadingStationPos() {
    return loadingStationPos;
  }

  public static void noNote() {
    SmartDashboard.putNumber("notePos", 0);
  }

  public static void noteHalfwayIn() {
    SmartDashboard.putNumber("notePos", 1);
  }

  public static void noteIn() {
    SmartDashboard.putNumber("notePos", 2);
  }

  public static void amplifiedStart() {
    isAmplified = true;
    amplifiedTimeLeft = 10.0;
    timer.reset();
    timer.start();
  }

  public void periodic() {
    if (isAmplified) {
      amplifiedTimeLeft = 10.0 - timer.get();
      SmartDashboard.putNumber("ampTimeLeft", amplifiedTimeLeft);
      if (amplifiedTimeLeft < 0.0) {
        isAmplified = false;
        timer.stop();
      }
    }
  }
}