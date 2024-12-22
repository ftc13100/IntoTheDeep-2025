package org.firstinspires.ftc.teamcode.opModes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@TeleOp
@Config
class IntakeTuner : CommandOpMode() {
    private lateinit var intake: Servo
    private lateinit var intakeBelt: Servo

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        intake = hardwareMap[Servo::class.java, ControlBoard.INTAKE.deviceName]
        intakeBelt = hardwareMap[Servo::class.java, ControlBoard.INTAKE_BELT.deviceName]

        IntakeSubsystem.initialize(hardwareMap)

        RunCommand({
            IntakeSubsystem.setClaw(claw)
        }).perpetually().schedule()

        RunCommand({
            IntakeSubsystem.target = wristTarget
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Intake Position", IntakeSubsystem.position)
            telemetry.addData("Intake Velocity", IntakeSubsystem.velocity)

            telemetry.update()
        }).perpetually().schedule()
    }

    companion object {
        @JvmField var claw = 0.75
        @JvmField var wristTarget = 0.0
    }
}