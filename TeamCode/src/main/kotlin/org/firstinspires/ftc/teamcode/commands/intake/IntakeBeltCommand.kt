package org.firstinspires.ftc.teamcode.commands.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

class IntakeBeltCommand(
    private val intakeBelt: Boolean,
    private val intakeSubsystem: IntakeSubsystem
) : CommandBase() {
    override fun execute() {
        if (intakeBelt) {
            intakeSubsystem.intakePos()
        }
        else {
            intakeSubsystem.outtakePos()
        }
    }
}