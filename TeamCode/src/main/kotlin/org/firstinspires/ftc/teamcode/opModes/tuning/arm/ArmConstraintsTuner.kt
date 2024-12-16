package org.firstinspires.ftc.teamcode.opModes.tuning.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous
class ArmConstraintsTuner : LinearOpMode() {
    private var maxVel = 0.0
    private var maxAccel = 0.0

    private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        ArmSubsystem.initialize(hardwareMap)

        waitForStart()

        var dt = 0.0
        var prev = 0.0

        timer.reset()
        while (opModeIsActive()) {
            while (timer.seconds() < TIME_TO_RUN) {
                ArmSubsystem.clockwise()

                dt = timer.seconds() - dt

                val curr = ElevatorSubsystem.velocity
                val dv = curr - prev

                maxVel = ArmSubsystem.velocity.coerceAtLeast(maxVel)
                maxAccel = maxAccel.coerceAtLeast(dv / dt)

                prev = curr

                telemetry.addData("Velocity", ElevatorSubsystem.velocity)
                telemetry.addData("Position", ElevatorSubsystem.position)

                telemetry.addData("Max Velocity", maxVel)
                telemetry.addData("Max Acceleration", maxAccel)

                telemetry.update()

                if (isStopRequested) break
            }

            telemetry.addData("Final Max Velocity", maxVel)
            telemetry.addData("Final Max Acceleration", maxAccel)
            telemetry.update()
        }
    }

    companion object {
        @JvmField
        var TIME_TO_RUN = 0.7
    }
}