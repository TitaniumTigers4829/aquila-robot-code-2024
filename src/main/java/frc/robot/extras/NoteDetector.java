// Copyright (c) LukeLib
package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class NoteDetector {

  private static Pose2d notePos;
  private final BilinearInterpolator pixelLookupInterpolator;

  public NoteDetector() {
    pixelLookupInterpolator = new BilinearInterpolator(VisionConstants.noteDetectionLookupTable);
  }

  public static double[] update() {
    double x_center = SmartDashboard.getNumber("EllipseCenterX", -1.0); //0.5 * camera's width for resolution in pixels
    double y_center = SmartDashboard.getNumber("EllipseCenterY",-1.0); //0.5 * camera's height for resolution in pixels

    return new double[]{x_center, y_center};
  }

  /**
   * This gets the position of the nearest note, relative to the robot and the direction.
   * its facing.
   * @return the offset of the note relative to the robot
   */
  public Pose2d getNoteRobotRelativeOffset() {
    double[] data = update();
    if (data[0] != -1.0 && data[1] != -1.0) {
    double[] worldPts = pixelLookupInterpolator.getLookupValue(data[0], data[1]);
    notePos = new Pose2d(worldPts[0], worldPts[1],new Rotation2d()); //fixed so that the x gets the distance from center of screen
    return notePos;
    }
    else{
      return new Pose2d(-1.0,-1.0, new Rotation2d());
    }
  }

  public Translation2d applyCameraOffset(Pose2d notePos) {
    return new Translation2d(notePos.getX(), notePos.getY() - Units.inchesToMeters(2));
  }
}
