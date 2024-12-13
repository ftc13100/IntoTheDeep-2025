package org.firstinspires.ftc.teamcode.opModes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeBeltSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@TeleOp
@Config
class IntakeTuner : CommandOpMode() {
    private lateinit var intake: Servo
    private lateinit var intakeBelt: Servo

    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var intakeBeltSubsystem: IntakeBeltSubsystem

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        intake = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE.deviceName)
        intakeBelt = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE_BELT.deviceName)

        intakeSubsystem = IntakeSubsystem(intake)
        intakeBeltSubsystem = IntakeBeltSubsystem(intakeBelt)

        RunCommand({
            intakeSubsystem.setSpeed(claw)
        }).perpetually().schedule()

        RunCommand({
            intakeBeltSubsystem.setPos(belt)
        }).perpetually().schedule()

    }

    companion object {
        @JvmField
        var claw = 0.75

        @JvmField
        var belt = 0.0
    }
}