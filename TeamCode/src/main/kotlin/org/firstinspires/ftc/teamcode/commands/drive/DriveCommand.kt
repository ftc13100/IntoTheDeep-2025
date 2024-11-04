package org.firstinspires.ftc.teamcode.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.pow

class DriveCommand(
    private val subsystem: DriveSubsystem,
    private val leftX: DoubleSupplier,
    private val leftY: DoubleSupplier,
    private val rightX: DoubleSupplier,
    private val zoneVal: Double,
) : CommandBase() {
    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.drive(
            leftY = zonedDrive(-leftY.asDouble, zoneVal).pow(3),
            leftX = zonedDrive(leftX.asDouble, zoneVal).pow(3),
            rightX = zonedDrive(rightX.asDouble, zoneVal).pow(3),
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
