
package frc.robot.wmlib2.sensor

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.wmlib2.sensor.IO_Gyro
import frc.robot.wmlib2.sensor.IO_Gyro.GyroIOInputs
import edu.wpi.first.wpilibj.SPI;

class IO_Navx(
) : IO_Gyro{

    val ahrs = AHRS(SPI.Port.kMXP)

    init{
        ahrs.calibrate()
        ahrs.reset()
    }

    fun getRotation2dWrapped(): Rotation2d{
        val degrees = if((ahrs.yaw.toDouble() % 360) < 0.0){(ahrs.yaw.toDouble() % 360) + 360}else{(ahrs.yaw.toDouble() % 360)}
        return Rotation2d.fromDegrees(-degrees)
    }

    override fun updateInputs(inputs: GyroIOInputs){
        inputs.connected = ahrs.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-ahrs.yaw.toDouble())
    }

}
