
package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable

interface ModuleIO{

    class ModuleIOInputs : LoggableInputs {
        var drivePositionRad = 0.0
        var driveVelocityRadPerSec = 0.0
        var driveAppliedVolts = 0.0
        var driveCurrentAmps: DoubleArray = doubleArrayOf()

        var turnAbsolutePosition: Rotation2d = Rotation2d()
        var turnPosition: Rotation2d = Rotation2d()
        var turnVelocityRadPerSec = 0.0
        var turnAppliedVolts = 0.0
        var turnCurrentAmps: DoubleArray = doubleArrayOf()

        // Send class values to log
        override fun toLog(table: LogTable) {
            table.put("drivePositionRad", drivePositionRad)
            table.put("driveVelocityRadPerSec", driveVelocityRadPerSec)
            table.put("driveAppliedVolts", driveAppliedVolts)
            // Add similar lines for the rest of the properties
        }

        // Retrieve class values from log
        override fun fromLog(table: LogTable) {
            drivePositionRad = table.get("drivePositionRad", drivePositionRad)
            driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec)
            driveAppliedVolts = table.get("driveAppliedVolts", driveAppliedVolts)
            // Add similar lines for the rest of the properties
        }
    }

    // Updates the set of loggable inputs.
    fun updateInputs(inputs: ModuleIOInputs) {}

    // Run the drive motor at the specified voltage.
    fun setDriveVoltage(volts: Double) {}

    // Run the turn motor at the specified voltage.
    fun setTurnVoltage(volts: Double) {}

    // Enable or disable brake mode on the drive motor.
    fun setDriveBrakeMode(enable: Boolean) {}

    // Enable or disable brake mode on the turn motor.
    fun setTurnBrakeMode(enable: Boolean) {}
}

