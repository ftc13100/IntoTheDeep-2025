package org.firstinspires.ftc.teamcode.commands.elevator

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

/**
 * Elevator Command to set PID position for the elevator
 * @see ElevatorSubsystem
 */
class ElevatorCommand(
    private val setpoint: Double,
    private val subsystem: ElevatorSubsystem,
) : CommandBase() {
    override fun initialize() {
        subsystem.setpoint = setpoint
        addRequirements(ElevatorSubsystem)
    }

    override fun execute() = subsystem.operateElevator()

    override fun isFinished(): Boolean = subsystem.isBusy

    override fun end(interrupted: Boolean) = subsystem.stop()
}