package org.firstinspires.ftc.teamcode.opModes.tuning.roadrunner

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

/**
 * X / ▢         - Front Left
 * Y / Δ         - Front Right
 * B / O         - Rear  Right
 * A / X         - Rear  Left
 * The buttons are mapped to match the wheels spatially if you
 * were to rotate the gamepad 45deg°. x/square is the front left
 * ________        and each button corresponds to the wheel as you go clockwise
 * / ______ \
 * ------------.-'   _  '-..+              Front of Bot
 * /   _  ( Y )  _  \                  ^
 * |  ( X )  _  ( B ) |     Front Left   \    Front Right
 * ___  '.      ( A )     /|       Wheel       \      Wheel
 * .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
 * |       |                 |                      \
 * '.___.' '.               |          Rear Left    \   Rear Right
 * '.             /             Wheel       \    Wheel
 * \.          .'              (A/X)        \   (B/O)
 * \________/
 *
 */
@Disabled
@Config
@TeleOp(group = "drive")
class MotorDirectionDebugger : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val drive = DriveSubsystem(hardwareMap)
        telemetry.addLine("Press play to begin the debugging opmode")
        telemetry.update()
        waitForStart()
        if (isStopRequested) return
        telemetry.clearAll()
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        while (!isStopRequested) {
            telemetry.addLine("Press each button to turn on its respective motor")
            telemetry.addLine()
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - MotorPDController</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>")
            telemetry.addLine()
            if (gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0.0, 0.0, 0.0)
                telemetry.addLine("Running MotorPDController: Front Left")
            } else if (gamepad1.y) {
                drive.setMotorPowers(0.0, 0.0, 0.0, MOTOR_POWER)
                telemetry.addLine("Running MotorPDController: Front Right")
            } else if (gamepad1.b) {
                drive.setMotorPowers(0.0, 0.0, MOTOR_POWER, 0.0)
                telemetry.addLine("Running MotorPDController: Rear Right")
            } else if (gamepad1.a) {
                drive.setMotorPowers(0.0, MOTOR_POWER, 0.0, 0.0)
                telemetry.addLine("Running MotorPDController: Rear Left")
            } else {
                drive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
                telemetry.addLine("Running MotorPDController: None")
            }
            telemetry.update()
        }
    }

    companion object {
        var MOTOR_POWER = 0.7
    }
}