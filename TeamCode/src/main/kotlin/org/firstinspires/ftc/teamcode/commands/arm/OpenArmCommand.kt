package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.arm.OpenArmSubsystem

class OpenArmCommand(
    private val subsystem: OpenArmSubsystem,
    private val turn: Boolean,
) : CommandBase() {

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