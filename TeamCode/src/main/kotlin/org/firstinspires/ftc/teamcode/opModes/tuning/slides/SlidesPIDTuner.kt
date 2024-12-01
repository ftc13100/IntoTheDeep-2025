package org.firstinspires.ftc.teamcode.opModes.tuning.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.OpenArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@TeleOp
@Config
class SlidesPIDTuner : CommandOpMode() {
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor

    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor

    private lateinit var armSubsystem: OpenArmSubsystem
    private lateinit var slidesSubsystem: ElevatorSubsystem

    private lateinit var armUpCommand: OpenArmCommand
    private lateinit var armDownCommand: OpenArmCommand

    private lateinit var operator: GamepadEx

    override fun initialize() {
        operator = GamepadEx(gamepad2)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_60)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_60)

        slidesLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName, Motor.GoBILDA.RPM_1150)
        slidesRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName, Motor.GoBILDA.RPM_1150)

        armSubsystem = OpenArmSubsystem(armRight, armLeft) { 0.0 }
        slidesSubsystem = ElevatorSubsystem(slidesRight, slidesLeft, armSubsystem::armAngle)

        armUpCommand = OpenArmCommand(armSubsystem, true)
        armDownCommand = OpenArmCommand(armSubsystem, false)

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(armUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(armDownCommand)

        RunCommand({
            slidesSubsystem.setpoint = target
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Slides Position", slidesSubsystem.slidePos)
            telemetry.addData("Setpoint", target)
            telemetry.addData("arm angle", armSubsystem.armAngle)
            telemetry.addData("slides Velocity", slidesSubsystem.slideVelocity)
            telemetry.update()
        }).perpetually().schedule()

        register(slidesSubsystem)
    }

    companion object {
        @JvmField
        var target = 0.0
    }
}