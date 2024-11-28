package org.firstinspires.ftc.teamcode.commands.elevator

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.OpenElevatorSubsystem

class SpinUpCommand(
    private val subsystem: OpenElevatorSubsystem
) : CommandBase() {

    override fun execute() {
        subsystem.up()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }


}