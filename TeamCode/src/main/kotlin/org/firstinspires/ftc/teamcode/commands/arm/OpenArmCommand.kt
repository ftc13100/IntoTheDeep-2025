package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

class OpenArmCommand(
    private val subsystem: ArmSubsystem,
    private val turn: Boolean,
) : CommandBase() {
    override fun initialize() {
        ArmSubsystem.enabled = false

        addRequirements(subsystem)
    }

    override fun execute() {
        if (turn) {
            subsystem.clockwise()
        } else {
            subsystem.anticlockwise()
        }
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }


}