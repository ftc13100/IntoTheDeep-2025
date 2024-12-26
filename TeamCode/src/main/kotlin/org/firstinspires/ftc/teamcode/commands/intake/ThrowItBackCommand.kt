package org.firstinspires.ftc.teamcode.commands.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

class ThrowItBackCommand(
    private val subsystem: IntakeSubsystem
) : CommandBase() {
    override fun execute() = subsystem.wristUp()

    override fun isFinished() = subsystem.isPressed

    override fun end(interrupted: Boolean) = subsystem.reset()
}