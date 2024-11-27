package org.firstinspires.ftc.teamcode.opModes.tuning.roadrunner

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.DriveConstants.getMotorVelocityF

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 *
 *
 * Upon pressing start, your bot will run at max power for RUNTIME seconds.
 *
 *
 * Further fine tuning of kF may be desired.
 */
//@Disabled //@Config
@Autonomous(group = "drive")
class MaxVelocityTuner : LinearOpMode() {
    private lateinit var timer: ElapsedTime
    private var maxVelocity = 0.0
    private lateinit var batteryVoltageSensor: VoltageSensor

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = DriveSubsystem(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.")
        telemetry.addLine("Please ensure you have enough space cleared.")
        telemetry.addLine("")
        telemetry.addLine("Press start when ready.")
        telemetry.update()

        waitForStart()

        telemetry.clearAll()
        telemetry.update()

        drive.setDrivePower(Pose2d(1.0, 0.0, 0.0))

        timer = ElapsedTime()

        while (!isStopRequested && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate()
            val poseVelo = requireNotNull(
                drive.poseVelocity
            ) { "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer." }

            maxVelocity = poseVelo.vec().norm().coerceAtLeast(maxVelocity)
        }

        drive.setDrivePower(Pose2d())

        val effectiveKf = getMotorVelocityF(veloInchesToTicks(maxVelocity))

        telemetry.addData("Max Velocity", maxVelocity)
        telemetry.addData(
            "Voltage Compensated kF",
            effectiveKf * batteryVoltageSensor.voltage / 12
        )

        telemetry.update()

        while (!isStopRequested && opModeIsActive()) idle()
    }

    private fun veloInchesToTicks(inchesPerSec: Double): Double {
        return inchesPerSec / (2 * Math.PI * DriveConstants.WHEEL_RADIUS) / DriveConstants.GEAR_RATIO * DriveConstants.TICKS_PER_REV
    }

    companion object {
        var RUNTIME = 2.0
    }
}