<<<<<<<< HEAD:TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/commands/slides/SpinDownCommand.kt
package org.firstinspires.ftc.teamcode.commands.slides
========
package org.firstinspires.ftc.teamcode.commands.elevator
>>>>>>>> 509c48b (Arm PID infra:):TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/commands/elevator/SpinDownCommand.kt

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem

class SpinDownCommand(
    private val subsystem: SlidesSubsystem
) : CommandBase() {


    override fun execute() {
        subsystem.down()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }
}