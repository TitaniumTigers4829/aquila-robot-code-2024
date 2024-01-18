// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SmarterDashboardRegistry {
  private static double[] pose = new double[]{0, 0, 0};
  private static double[] limelight_pose = new double[]{0, 0, 0};
  private static double[] orientation = new double[]{0, 0, 0};
  // isactive, alliance, setpoint (1=speaker, 2=amp)
  private static boolean[] pathData = new boolean[]{false, false, false};

  public static void setPose(Pose2d robotPose) {
    pose[0] = robotPose.getX();
    pose[1] = robotPose.getY();
    pose[2] = robotPose.getRotation().getDegrees();
    SmartDashboard.putNumberArray("botPose", pose);
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
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[1]));
  }

  public static Pose2d getLimelightPose() {
    return new Pose2d(limelight_pose[0], limelight_pose[1], Rotation2d.fromDegrees(limelight_pose[1]));
  }

  public static boolean[] getPathData() {
    return pathData;
  }
}
