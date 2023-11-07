// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.wmlib2.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import frc.robot.Constants
import frc.robot.subsystems.drive.ModuleIO
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs
import org.littletonrobotics.junction.Logger

class Module(private val io: ModuleIO, private val index: Int) {
    private val WHEEL_RADIUS = Units.inchesToMeters(2.0)


    private val inputs = ModuleIO.ModuleIOInputs()
    private val driveFeedforward: SimpleMotorFeedforward
    private val driveFeedback: PIDController
    private val turnFeedback: PIDController
    private var angleSetpoint: Rotation2d? = null // Setpoint for closed loop control, null for open loop
    private var speedSetpoint: Double? = null // Setpoint for closed loop control, null for open loop
    private var turnRelativeOffset: Rotation2d? = null // Relative + Offset = Absolute
    private var lastPositionMeters = 0.0 // Used for delta calculation

    init {
        // Switch constants based on mode (the physics simulator is treated as a separate robot with different tuning)
        when (Constants.currentMode) {
            Constants.Mode.REAL, Constants.Mode.REPLAY -> {
                driveFeedforward = SimpleMotorFeedforward(0.1, 0.13)
                driveFeedback = PIDController(0.05, 0.0, 0.0)
                turnFeedback = PIDController(7.0, 0.0, 0.0)
            }
            Constants.Mode.SIM -> {
                driveFeedforward = SimpleMotorFeedforward(0.0, 0.13)
                driveFeedback = PIDController(0.1, 0.0, 0.0)
                turnFeedback = PIDController(10.0, 0.0, 0.0)
            }
            else -> {
                driveFeedforward = SimpleMotorFeedforward(0.0, 0.0)
                driveFeedback = PIDController(0.0, 0.0, 0.0)
                turnFeedback = PIDController(0.0, 0.0, 0.0)
            }
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI)
        setBrakeMode(true)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/Module$index", inputs)

        // On the first cycle, reset relative turn encoder
        // Wait until the absolute angle is nonzero in case it wasn't initialized yet
        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition)
        }

        // Run closed loop turn control
        if (angleSetpoint != null) {
            io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), angleSetpoint!!.getRadians()))

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
                // Scale velocity based on turn error
                //
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                val adjustSpeedSetpoint = speedSetpoint!! * Math.cos(turnFeedback.getPositionError())

                // Run drive controller
                val velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS
                io.setDriveVoltage(
                    driveFeedforward.calculate(velocityRadPerSec) +
                            driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
                )
            }
        }
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    fun runSetpoint(state: SwerveModuleState): SwerveModuleState {
        // Optimize state based on the current angle
        // Controllers run in "periodic" when the setpoint is not null
        val optimizedState = SwerveModuleState.optimize(state, getAngle())

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle
        speedSetpoint = optimizedState.speedMetersPerSecond

        return optimizedState
    }

    /** Runs the module with the specified voltage while controlling to zero degrees. */
    fun runCharacterization(volts: Double) {
        // Closed loop turn control
        angleSetpoint = Rotation2d()

        // Open loop drive control
        io.setDriveVoltage(volts)
        speedSetpoint = null
    }

    /** Disables all outputs to motors. */
    fun stop() {
        io.setTurnVoltage(0.0)
        io.setDriveVoltage(0.0)

        // Disable closed loop control for turn and drive
        angleSetpoint = null
        speedSetpoint = null
    }

    /** Sets whether brake mode is enabled. */
    fun setBrakeMode(enabled: Boolean) {
        io.setDriveBrakeMode(enabled)
        io.setTurnBrakeMode(enabled)
    }

    /** Returns the current turn angle of the module. */
    fun getAngle(): Rotation2d {
        if (turnRelativeOffset == null) {
            return Rotation2d()
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset)
        }
    }

    /** Returns the current drive position of the module in meters. */
    fun getPositionMeters(): Double {
        return inputs.drivePositionRad * WHEEL_RADIUS
    }

    /** Returns the current drive velocity of the module in meters per second. */
    fun getVelocityMetersPerSec(): Double {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS
    }

    /** Returns the module position (turn angle and drive position). */
    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(getPositionMeters(), getAngle())
    }

    /** Returns the module position delta since the last call to this method. */
    fun getPositionDelta(): SwerveModulePosition {
        val delta = SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle())
        lastPositionMeters = getPositionMeters()
        return delta
    }

    /** Returns the module state (turn angle and drive velocity). */
    fun getState(): SwerveModuleState {
        return SwerveModuleState(getVelocityMetersPerSec(), getAngle())
    }

    /** Returns the drive velocity in radians/sec. */
    fun getCharacterizationVelocity(): Double {
        return inputs.driveVelocityRadPerSec
    }
}
