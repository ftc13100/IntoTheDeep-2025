package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.arm.DefaultArmCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

@TeleOp
@Config
class ArmTeleOp : CommandOpMode() {
    private lateinit var upCommand: ArmCommand
    private lateinit var downCommand: ArmCommand

    private lateinit var armCommand: DefaultArmCommand

    private lateinit var gamepad: GamepadEx

    override fun initialize() {
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        ArmSubsystem.initialize(hardwareMap, telemetry)

        armCommand = DefaultArmCommand(ArmSubsystem)

        RunCommand({
            ArmSubsystem.target = Math.toRadians(target)
        }).perpetually().schedule()

        ArmSubsystem.defaultCommand = armCommand
    }

    companion object {
        @JvmField var target = 0.0
    }
}