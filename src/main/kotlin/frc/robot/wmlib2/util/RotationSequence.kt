package frc.robot.wmlib2.util

import edu.wpi.first.math.geometry.Rotation2d
import java.util.Map.Entry
import java.util.TreeMap
import kotlin.math.pow

// Imported to kotlin, originally from https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/trajectory/RotationSequence.java

/**
 * Represents a sequence of timed rotations. The position and velocity of the robot is calculated to
 * minimize acceleration.
 */
class RotationSequence(private val sequence: TreeMap<Double, Rotation2d>) {

    /**
     * Sample the rotation sequence at a point in time.
     *
     * @param timeSeconds The point in time since the beginning of the rotation sequence to sample.
     * @return The state at that point in time.
     */
    fun sample(timeSeconds: Double): State {
        val positionRadians: Double
        val velocityRadiansPerSec: Double

        val lastPoint: Map.Entry<Double, Rotation2d>? = sequence.floorEntry(timeSeconds) as Map.Entry<Double, Rotation2d>?
        val nextPoint: Map.Entry<Double, Rotation2d>? = sequence.higherEntry(timeSeconds) as Map.Entry<Double, Rotation2d>?

        if (lastPoint == null && nextPoint == null) { // No points in sequence
            positionRadians = 0.0
            velocityRadiansPerSec = 0.0
        } else if (lastPoint == null) { // Before start of sequence
            positionRadians = nextPoint!!.value.radians
            velocityRadiansPerSec = 0.0
        } else if (nextPoint == null) { // Before end of sequence
            positionRadians = lastPoint.value.radians
            velocityRadiansPerSec = 0.0
        } else {
            val accelerationRadiansPerSec2 = (4 * nextPoint.value.minus(lastPoint.value).radians) /
                    (nextPoint.key - lastPoint.key).pow(2.0)

            if (timeSeconds < (nextPoint.key + lastPoint.key) / 2) { // Accelerating
                positionRadians =
                        lastPoint.value.radians + ((accelerationRadiansPerSec2 / 2) *
                                (timeSeconds - lastPoint.key).pow(2.0))
                velocityRadiansPerSec = (timeSeconds - lastPoint.key) * accelerationRadiansPerSec2

            } else { // Decelerating
                positionRadians =
                        nextPoint.value.radians - ((accelerationRadiansPerSec2 / 2) *
                                (timeSeconds - nextPoint.key).pow(2.0))
                velocityRadiansPerSec = (nextPoint.key - timeSeconds) * accelerationRadiansPerSec2
            }
        }

        // Keep position within acceptable range
        var posRadians = positionRadians
        while (posRadians > Math.PI) {
            posRadians -= Math.PI * 2
        }
        while (posRadians < -Math.PI) {
            posRadians += Math.PI * 2
        }

        return State(Rotation2d(posRadians), velocityRadiansPerSec)

    }

    /** Represents a state in a rotation sequence with a position and velocity. */
    data class State(var position: Rotation2d = Rotation2d(),
                     var velocityRadiansPerSec: Double = 0.0)
}
