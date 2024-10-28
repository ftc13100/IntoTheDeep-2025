package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.drive.SpinUpCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem

@TeleOp
class MainTeleOp: CommandOpMode() {

    private lateinit var leftSlideString : Motor
    private lateinit var rightSlideString : Motor


    private lateinit var slidesSubsystem: SlidesSubsystem


    private lateinit var spinUpCommand: SpinUpCommand
    private lateinit var spinDownCommand: SpinDownCommand


    private lateinit var driver : GamepadEx
    private lateinit var operator : GamepadEx

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        rightSlideString = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)
        leftSlideString = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)


        slidesSubsystem = SlidesSubsystem(rightSlideString, leftSlideString)


        spinUpCommand = SpinUpCommand(slidesSubsystem)
        spinDownCommand = SpinDownCommand(slidesSubsystem)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
    }
}