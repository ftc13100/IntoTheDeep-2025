package org.firstinspires.ftc.teamcode.commands.intake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

class IntakeBeltCommand(
    private val setpoint: Double,
    private val subsystem: IntakeSubsystem
) : CommandBase() {
    override fun initialize() {
        subsystem.target = setpoint
        addRequirements(subsystem)
    }

    override fun execute() = subsystem.operateWrist()

    override fun isFinished() = subsystem.isBusy

    override fun end(interrupted: Boolean) = subsystem.wristStop()
}