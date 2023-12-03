
package frc.robot.wmlib2.util

import edu.wpi.first.math.geometry.*

// Imported to kotlin, originally from https://github.com/Mechanical-Advantage/RobotCode2023/blob/9884d13b2220b76d430e82248fd837adbc4a10bc/src/main/java/org/littletonrobotics/frc2023/util/GeomUtil.java#L20

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
object GeomUtil {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    fun translationToTransform(translation: Translation2d): Transform2d {
        return Transform2d(translation, Rotation2d())
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x componenet of the translation
     * @param y The y componenet of the translation
     * @return The resulting transform
     */
    fun translationToTransform(x: Double, y: Double): Transform2d {
        return Transform2d(Translation2d(x, y), Rotation2d())
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    fun rotationToTransform(rotation: Rotation2d): Transform2d {
        return Transform2d(Translation2d(), rotation)
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    fun poseToTransform(pose: Pose2d): Transform2d {
        return Transform2d(pose.translation, pose.rotation)
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    fun transformToPose(transform: Transform2d): Pose2d {
        return Pose2d(transform.translation, transform.rotation)
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    fun translationToPose(translation: Translation2d): Pose2d {
        return Pose2d(translation, Rotation2d())
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    fun rotationToPose(rotation: Rotation2d): Pose2d {
        return Pose2d(Translation2d(), rotation)
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    fun multiplyTwist(twist: Twist2d, factor: Double): Twist2d {
        return Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor)
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    fun pose3dToTransform3d(pose: Pose3d): Transform3d {
        return Transform3d(pose.translation, pose.rotation)
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    fun transform3dToPose3d(transform: Transform3d): Pose3d {
        return Pose3d(transform.translation, transform.rotation)
    }

    /**
     * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Y). chain
     *
     * @param transform The original translation
     * @return The resulting translation
     */
    fun translation3dTo2dXY(translation: Translation3d): Translation2d {
        return Translation2d(translation.x, translation.y)
    }

    /**
     * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
     *
     * @param transform The original translation
     * @return The resulting translation
     */
    fun translation3dTo2dXZ(translation: Translation3d): Translation2d {
        return Translation2d(translation.x, translation.z)
    }
}
