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

package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
//import frc.robot.subsystems.drive.Drive
import java.util.function.DoubleSupplier

//object DriveCommands {
  //  private const val DEADBAND = 0.1

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param drive          The drive subsystem to control.
     * @param xSupplier      A supplier for the x-axis input.
     * @param ySupplier      A supplier for the y-axis input.
     * @param omegaSupplier  A supplier for the angular velocity input.
     * @return A command that executes the field-relative drive.

    fun joystickDrive(
        drive: Drive,
        xSupplier: DoubleSupplier,
        ySupplier: DoubleSupplier,
        omegaSupplier: DoubleSupplier
    ): Command {
        return Commands.run(
            {
                // Apply deadband
                val linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(xSupplier.asDouble, ySupplier.asDouble),
                    DEADBAND
                )
                val linearDirection = Rotation2d(xSupplier.asDouble, ySupplier.asDouble)
                val omega = MathUtil.applyDeadband(omegaSupplier.asDouble, DEADBAND)

                // Square values
                val squaredLinearMagnitude = linearMagnitude * linearMagnitude
                val squaredOmega = Math.copySign(omega * omega, omega)

                // Calculate new linear velocity
                val linearVelocity = Pose2d(Translation2d(), linearDirection)
                    .transformBy(Transform2d(0.0, 0.0))
                    .translation


                // Convert to field relative speeds & send command
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.x * drive.maxLinearSpeedMetersPerSec,
                        linearVelocity.y * drive.maxLinearSpeedMetersPerSec,
                        squaredOmega * drive.maxAngularSpeedRadPerSec,
                        drive.rotation
                    )
                )
            },
            drive
        )
    }
}

     */