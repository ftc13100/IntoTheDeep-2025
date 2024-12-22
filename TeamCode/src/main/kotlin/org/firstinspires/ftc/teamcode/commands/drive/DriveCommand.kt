package org.firstinspires.ftc.teamcode.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import kotlin.math.pow

class DriveCommand(
    private val subsystem: DriveSubsystem,
    private val leftX: () -> Double,
    private val leftY: () -> Double,
    private val rightX: () -> Double,
    private val zoneVal: Double,
    private val multipier: Double = 0.9,
) : CommandBase() {
    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.drive(
            leftY = zonedDrive(leftY.invoke() * multipier, zoneVal).pow(3),
            leftX = zonedDrive(leftX.invoke() * multipier, zoneVal).pow(3),
            rightX = zonedDrive(rightX.invoke() * multipier, zoneVal).pow(3),
        )
    }

    private fun zonedDrive(drive: Double, zoneVal: Double) =
        if (drive in -zoneVal..zoneVal) {
            0.0
        } else if (drive > zoneVal) {
            drive / (1 - zoneVal) - zoneVal / (1 - zoneVal)
        } else {
            drive / (1 - zoneVal) + zoneVal / (1 - zoneVal)
        }
}
