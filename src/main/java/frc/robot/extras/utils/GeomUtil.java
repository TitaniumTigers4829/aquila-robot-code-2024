// inspired/copied from 6328

package com.igknighters.util.geom;

import com.igknighters.constants.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Geometry utilities for working with translations, rotations, transforms, and
 * poses.
 */
public class GeomUtil {
    public static final Translation2d TRANSLATION2D_ZERO = new Translation2d();
    public static final Translation2d TRANSLATION2D_CENTER = new Translation2d(
        FieldConstants.FIELD_LENGTH / 2.0,
        FieldConstants.FIELD_WIDTH / 2.0
    );
    public static final Rotation2d ROTATION2D_ZERO = new Rotation2d();
    public static final Rotation2d ROTATION2D_PI = Rotation2d.fromDegrees(180.0);
    public static final Pose2d POSE2D_ZERO = new Pose2d();
    public static final Pose2d POSE2D_CENTER = new Pose2d(TRANSLATION2D_CENTER, ROTATION2D_ZERO);
    public static final Transform2d TRANSFORM2D_ZERO = new Transform2d();
    public static final Translation3d TRANSLATION3D_ZERO = new Translation3d();
    public static final Rotation2d ROTATION3D_ZERO = new Rotation2d();
    public static final Transform3d TRANSFORM3D_ZERO = new Transform3d();
    public static final Pose3d POSE3D_ZERO = new Pose3d();
    public static final Twist2d TWIST2D_ZERO = new Twist2d();
    public static final Twist3d TWIST3D_ZERO = new Twist3d();

    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Rotation2d rotation) {
        return new Transform2d(new Translation2d(), rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
                pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start
     * of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist  The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiply(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start
     * of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z).
     * chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Gets the angle between two points
     * 
     * @param currentTrans The current translation
     * @param pose The pose to get the angle to
     * @param angleOffet An offset to add to the angle
     * @return The angle between the two points
     */
    public static Rotation2d rotationRelativeToPose(Translation2d currentTrans, Translation2d pose) {
        double angleBetween = Math.atan2(
                pose.getY() - currentTrans.getY(),
                pose.getX() - currentTrans.getX());
        return Rotation2d.fromRadians(angleBetween);
    }
}