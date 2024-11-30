package org.firstinspires.ftc.teamcode.commands.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeBeltSubsystem

class IntakeBeltCommand(
    private val intakeBelt: Boolean,
    private val intakeBeltSubsystem: IntakeBeltSubsystem
) : CommandBase() {
    override fun execute() {
        if (intakeBelt) {
            intakeBeltSubsystem.intakePos()
        }
        else {
            intakeBeltSubsystem.outtakePos()
        }
    }
}