package org.firstinspires.ftc.teamcode.opModes.tuning.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@TeleOp
@Config
class SlidesPIDTuner : CommandOpMode() {
    private lateinit var armUpCommand: OpenArmCommand
    private lateinit var armDownCommand: OpenArmCommand

    private lateinit var operator: GamepadEx

    override fun initialize() {
        operator = GamepadEx(gamepad2)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        ArmSubsystem.initialize(hardwareMap)
        ElevatorSubsystem.initialize(hardwareMap)

        armUpCommand = OpenArmCommand(ArmSubsystem, true)
        armDownCommand = OpenArmCommand(ArmSubsystem, false)

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(armUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(armDownCommand)

        RunCommand({
            ElevatorSubsystem.setpoint = target
            ElevatorSubsystem.operateElevator()
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Slides Position", ElevatorSubsystem.position)
            telemetry.addData("Setpoint", target)

            telemetry.addData("Arm Angle", ArmSubsystem.angle)
            telemetry.addData("Slides Velocity", ElevatorSubsystem.velocity)

            telemetry.update()
        }).perpetually().schedule()

        register(ElevatorSubsystem)
    }

    companion object {
        @JvmField
        var target = 0.0
    }
}