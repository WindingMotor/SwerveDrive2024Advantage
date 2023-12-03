package frc.robot.wmlib2.util

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import java.util.*


// Converted to kotlin, originally from https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/AllianceFlipUtil.java#L24

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
object AllianceFlipUtil {
    /** Flips a translation to the correct side of the field based on the current alliance color. */
    fun apply(translation: Translation2d): Translation2d {
        return if (shouldFlip()) {
            Translation2d(FieldConstants.fieldLength - translation.x, translation.y)
        } else {
            translation
        }
    }

    /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
    fun apply(xCoordinate: Double): Double {
        return if (shouldFlip()) {
            FieldConstants.fieldLength - xCoordinate
        } else {
            xCoordinate
        }
    }

    /** Flips a rotation based on the current alliance color. */
    fun apply(rotation: Rotation2d): Rotation2d {
        return if (shouldFlip()) {
            Rotation2d(-rotation.cos, rotation.sin)
        } else {
            rotation
        }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    fun apply(pose: Pose2d): Pose2d {
        return if (shouldFlip()) {
            Pose2d(
                    FieldConstants.fieldLength - pose.x,
                    pose.y,
                    Rotation2d(-pose.rotation.cos, pose.rotation.sin)
            )
        } else {
            pose
        }
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the current alliance color.
     */
    fun apply(state: Trajectory.State): Trajectory.State {
        return if (shouldFlip()) {
            Trajectory.State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    Pose2d(
                            FieldConstants.fieldLength - state.poseMeters.x,
                            state.poseMeters.y,
                            Rotation2d(-state.poseMeters.rotation.cos, state.poseMeters.rotation.sin)
                    ),
                    -state.curvatureRadPerMeter
            )
        } else {
            state
        }
    }

    /** Flips a rotation sequence state based on the current alliance color. */
    fun apply(state: RotationSequence.State): RotationSequence.State {
        return if (shouldFlip()) {
            RotationSequence.State(
                    Rotation2d(-state.position.cos, state.position.sin),
                    -state.velocityRadiansPerSec
            )
        } else {
            state
        }
    }

    private fun shouldFlip(): Boolean {
        val allianceOptional: Optional<Alliance> = DriverStation.getAlliance()
        return allianceOptional.isPresent && allianceOptional.get() == Alliance.Red
    }

}
