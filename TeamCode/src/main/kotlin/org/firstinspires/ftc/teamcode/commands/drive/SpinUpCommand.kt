package org.firstinspires.ftc.teamcode.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem

class SpinUpCommand (
        private val subsystem: SlidesSubsystem
) : CommandBase() {

    override fun execute() {
        subsystem.up()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }


}