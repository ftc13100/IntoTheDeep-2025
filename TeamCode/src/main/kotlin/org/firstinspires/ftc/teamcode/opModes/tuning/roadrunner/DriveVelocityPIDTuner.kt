package org.firstinspires.ftc.teamcode.opModes.tuning.roadrunner

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator.generateSimpleMotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.DriveConstants

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 *
 * Recommended tuning process:
 *
 * 1. Increase kP until any phase lag is eliminated. Concurrently increase kD as necessary to
 *    mitigate oscillations.
 * 2. Add kI (or adjust kF) until the steady state/constant velocity plateaus are reached.
 * 3. Back off kP and kD a little until the response is less oscillatory (but without lag).
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
@Disabled
@Config
@Autonomous(group = "drive")
class DriveVelocityPIDTuner : LinearOpMode() {
    override fun runOpMode() {
        if (!DriveConstants.RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg(
                "%s does not need to be run if the built-in motor velocity" +
                        "PID is not in use", javaClass.simpleName
            )
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val drive = DriveSubsystem(hardwareMap)
        var mode = Mode.TUNING_MODE
        var lastKp = DriveConstants.MOTOR_VELO_PID.p
        var lastKi = DriveConstants.MOTOR_VELO_PID.i
        var lastKd = DriveConstants.MOTOR_VELO_PID.d
        var lastKf = DriveConstants.MOTOR_VELO_PID.f
        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID)
        val clock = NanoClock.system()
        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return
        var movingForwards = true
        var activeProfile = generateProfile(true)
        var profileStart = clock.seconds()
        while (!isStopRequested) {
            telemetry.addData("mode", mode)
            when (mode) {
                Mode.TUNING_MODE -> {
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    }

                    // calculate and set the motor power
                    val profileTime = clock.seconds() - profileStart
                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards
                        activeProfile = generateProfile(movingForwards)
                        profileStart = clock.seconds()
                    }
                    val motionState = activeProfile[profileTime]
                    val targetPower = DriveConstants.kV * motionState.v
                    drive.setDrivePower(Pose2d(targetPower, 0.0, 0.0))
                    val velocities = drive.getWheelVelocities()

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.v)
                    var i = 0
                    while (i < velocities.size) {
                        telemetry.addData("measuredVelocity$i", velocities[i])
                        telemetry.addData(
                            "error$i",
                            motionState.v - velocities[i]
                        )
                        i++
                    }
                }

                Mode.DRIVER_MODE -> {
                    if (gamepad1.b) {
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
                        mode = Mode.TUNING_MODE
                        movingForwards = true
                        activeProfile = generateProfile(movingForwards)
                        profileStart = clock.seconds()
                    }
                    drive.setWeightedDrivePower(
                        Pose2d(
                            -gamepad1.left_stick_y.toDouble(),
                            -gamepad1.left_stick_x.toDouble(),
                            -gamepad1.right_stick_x.toDouble()
                        )
                    )
                }
            }
            if (lastKp != DriveConstants.MOTOR_VELO_PID.p || lastKd != DriveConstants.MOTOR_VELO_PID.d || lastKi != DriveConstants.MOTOR_VELO_PID.i || lastKf != DriveConstants.MOTOR_VELO_PID.f) {
                drive.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    DriveConstants.MOTOR_VELO_PID
                )
                lastKp = DriveConstants.MOTOR_VELO_PID.p
                lastKi = DriveConstants.MOTOR_VELO_PID.i
                lastKd = DriveConstants.MOTOR_VELO_PID.d
                lastKf = DriveConstants.MOTOR_VELO_PID.f
            }
            telemetry.update()
        }
    }

    internal enum class Mode {
        DRIVER_MODE, TUNING_MODE
    }

    companion object {
        var DISTANCE = 72.0 // in
        private fun generateProfile(movingForward: Boolean): MotionProfile {
            val start = MotionState(if (movingForward) 0.0 else DISTANCE, 0.0, 0.0, 0.0)
            val goal = MotionState(if (movingForward) DISTANCE else 0.0, 0.0, 0.0, 0.0)
            return generateSimpleMotionProfile(
                start,
                goal,
                DriveConstants.MAX_VEL,
                DriveConstants.MAX_ACCEL
            )
        }
    }
}