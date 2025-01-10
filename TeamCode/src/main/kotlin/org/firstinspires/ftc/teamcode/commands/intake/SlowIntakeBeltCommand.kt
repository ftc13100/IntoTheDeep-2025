package org.firstinspires.ftc.teamcode.commands.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem


class SlowIntakeBeltCommand(
    private val subsystem: IntakeSubsystem,
    private val turn: Boolean,

) : CommandBase() {

        override fun initialize() {
            addRequirements(subsystem)
        }

        override fun execute() {
            if (turn) {
                subsystem.wristUpSlow()
            } else {
                subsystem.wristDownSlow()
            }
        }

        override fun end(interrupted: Boolean) {
            subsystem.wristStop()
        }


    }
