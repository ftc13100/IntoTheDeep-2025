package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.Subsystem

/**
 * Generic command that wraps Roadrunner 1.0 Actions
 */
class ActionCommand(
    private val action: Action,
    private vararg val subsystem: Subsystem,
    private val endBehavior: () -> Unit = { },
) : CommandBase() {
    var finished = false

    override fun initialize() = addRequirements(*subsystem)

    override fun execute() {
        val packet = TelemetryPacket()

        action.preview(packet.fieldOverlay())
        finished = !action.run(packet)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    override fun isFinished() = finished

    override fun end(interrupted: Boolean) = endBehavior.invoke()
}