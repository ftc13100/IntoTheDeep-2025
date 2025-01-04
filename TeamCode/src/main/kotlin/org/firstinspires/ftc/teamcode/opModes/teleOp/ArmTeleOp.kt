package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

@TeleOp
@Config
class ArmTeleOp : CommandOpMode() {
    private lateinit var upCommand: ArmCommand
    private lateinit var downCommand: ArmCommand

    private lateinit var gamepad: GamepadEx

    override fun initialize() {
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)

        ArmSubsystem.initialize(hardwareMap, telemetry)

        upCommand = ArmCommand(
            Math.toRadians(90.0),
            ArmSubsystem
        )

        downCommand = ArmCommand(
            Math.toRadians(0.0),
            ArmSubsystem
        )

        gamepad = GamepadEx(gamepad2)

        gamepad.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(upCommand, downCommand)
    }
}