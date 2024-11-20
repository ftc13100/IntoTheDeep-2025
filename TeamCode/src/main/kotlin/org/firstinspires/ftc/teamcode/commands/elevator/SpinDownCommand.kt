package org.firstinspires.ftc.teamcode.commands.elevator

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.OpenSlidesSubsystem

class SpinDownCommand(
    private val subsystem: OpenSlidesSubsystem
) : CommandBase() {


    override fun execute() {
        subsystem.down()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }
}