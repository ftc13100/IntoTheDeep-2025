package org.firstinspires.ftc.teamcode.commands.elevator

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

class ElevatorCommand(
    private val subsystem: ElevatorSubsystem,
    private val setpoint: Double
) : CommandBase() {
    override fun initialize() {
        subsystem.setpoint = setpoint
    }

    override fun execute() = subsystem.operateElevator()

    override fun isFinished(): Boolean = subsystem.isBusy

    override fun end(interrupted: Boolean) = subsystem.stop()
}