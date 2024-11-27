package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

class ArmCommand(
    private val setpoint: Double,
    private val subsystem: ArmSubsystem
) : CommandBase() {
    override fun initialize() {
        subsystem.setpoint = setpoint
    }
}