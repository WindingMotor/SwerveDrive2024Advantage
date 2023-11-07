
package frc.robot.wmlib2.swerve

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs
import frc.robot.subsystems.drive.ModuleIO

class ModuleIOSparkMax(private val index: Int) : ModuleIO {
    // Gear ratios for SDS MK4i L2, adjust as necessary
    private val DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
    private val TURN_GEAR_RATIO = 150.0 / 7.0

    private val driveSparkMax: CANSparkMax
    private val turnSparkMax: CANSparkMax

    private val driveEncoder: RelativeEncoder
    private val turnRelativeEncoder: RelativeEncoder
    private val turnAbsoluteEncoder: AnalogInput
    private val isTurnMotorInverted = true
    private val absoluteEncoderOffset: Rotation2d

    init {
        when (index) {
            0 -> {
                driveSparkMax = CANSparkMax(1, MotorType.kBrushless)
                turnSparkMax = CANSparkMax(2, MotorType.kBrushless)
                turnAbsoluteEncoder = AnalogInput(0)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }
            1 -> {
                driveSparkMax = CANSparkMax(3, MotorType.kBrushless)
                turnSparkMax = CANSparkMax(4, MotorType.kBrushless)
                turnAbsoluteEncoder = AnalogInput(1)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }
            2 -> {
                driveSparkMax = CANSparkMax(5, MotorType.kBrushless)
                turnSparkMax = CANSparkMax(6, MotorType.kBrushless)
                turnAbsoluteEncoder = AnalogInput(2)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }
            3 -> {
                driveSparkMax = CANSparkMax(7, MotorType.kBrushless)
                turnSparkMax = CANSparkMax(8, MotorType.kBrushless)
                turnAbsoluteEncoder = AnalogInput(3)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }
            else -> throw RuntimeException("Invalid module index")
        }

        driveSparkMax.restoreFactoryDefaults()
        turnSparkMax.restoreFactoryDefaults()

        driveSparkMax.setCANTimeout(250)
        turnSparkMax.setCANTimeout(250)

        driveEncoder = driveSparkMax.encoder
        turnRelativeEncoder = turnSparkMax.encoder

        turnSparkMax.inverted = isTurnMotorInverted
        driveSparkMax.setSmartCurrentLimit(40)
        turnSparkMax.setSmartCurrentLimit(30)
        driveSparkMax.enableVoltageCompensation(12.0)
        turnSparkMax.enableVoltageCompensation(12.0)

        driveEncoder.position = 0.0
        driveEncoder.measurementPeriod = 10
        driveEncoder.averageDepth = 2

        turnRelativeEncoder.position = 0.0
        turnRelativeEncoder.measurementPeriod = 10
        turnRelativeEncoder.averageDepth = 2

        driveSparkMax.setCANTimeout(0)
        turnSparkMax.setCANTimeout(0)

        driveSparkMax.burnFlash()
        turnSparkMax.burnFlash()
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.drivePositionRad =
            Units.rotationsToRadians(driveEncoder.position / DRIVE_GEAR_RATIO)
        inputs.driveVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.velocity / DRIVE_GEAR_RATIO)
        inputs.driveAppliedVolts = driveSparkMax.appliedOutput * driveSparkMax.busVoltage
        inputs.driveCurrentAmps = doubleArrayOf(driveSparkMax.outputCurrent)

        inputs.turnAbsolutePosition =
            Rotation2d(
                turnAbsoluteEncoder.voltage / RobotController.getVoltage5V() * 2.0 * Math.PI
            ).minus(absoluteEncoderOffset)
        inputs.turnPosition =
            Rotation2d.fromRotations(turnRelativeEncoder.position / TURN_GEAR_RATIO)
        inputs.turnVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.velocity / TURN_GEAR_RATIO)
        inputs.turnAppliedVolts = turnSparkMax.appliedOutput * turnSparkMax.busVoltage
        inputs.turnCurrentAmps = doubleArrayOf(turnSparkMax.outputCurrent)
    }

}
