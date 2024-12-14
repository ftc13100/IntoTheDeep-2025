package org.firstinspires.ftc.teamcode.commands.elevator

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

class SpinUpCommand(
    private val subsystem: ElevatorSubsystem
) : CommandBase() {

    override fun execute() {
        subsystem.spinUp()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }


}