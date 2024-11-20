package org.firstinspires.ftc.teamcode.commands.slides

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem

class SpinDownCommand (
        private val subsystem: SlidesSubsystem
) : CommandBase() {


    override fun execute() {
        subsystem.down()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }
}