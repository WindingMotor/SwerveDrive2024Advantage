
package frc.robot.subsystems.sensor

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.sensor.GyroIO
import frc.robot.subsystems.sensor.GyroIO.GyroIOInputs

class GyroPidgeonIO : GyroIO {
    private val pigeon = Pigeon2(20)
    private val yaw: StatusSignal<Double> = pigeon.yaw
    private val yawVelocity: StatusSignal<Double> = pigeon.angularVelocityZ

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)
        yaw.setUpdateFrequency(100.0)
        yawVelocity.setUpdateFrequency(100.0)
        pigeon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
        inputs.yawPosition = Rotation2d.fromDegrees(-yaw.valueAsDouble)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)
    }
}
