package org.firstinspires.ftc.teamcode.opModes.tuning.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@Autonomous
class IntakeConstraintsTuner : LinearOpMode() {
    private var maxVel = 0.0
    private var maxAccel = 0.0

    private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        IntakeSubsystem.initialize(hardwareMap)

        waitForStart()

        var dt = 0.0
        var prev = 0.0

        timer.reset()
        while (opModeIsActive()) {
            while (timer.seconds() < TIME_TO_RUN) {
                IntakeSubsystem.wristUp()

                dt = timer.seconds() - dt

                val curr = IntakeSubsystem.velocity
                val dv = curr - prev

                maxVel = IntakeSubsystem.velocity.coerceAtLeast(maxVel)
                maxAccel = maxAccel.coerceAtLeast(dv / dt)

                prev = curr

                telemetry.addData("Velocity", IntakeSubsystem.velocity)
                telemetry.addData("Angle", IntakeSubsystem.position)

                telemetry.addData("Max Velocity", maxVel)
                telemetry.addData("Max Acceleration", maxAccel)

                telemetry.update()

                if (isStopRequested) break
            }
            ArmSubsystem.stop()

            telemetry.addData("Final Max Velocity", maxVel)
            telemetry.addData("Final Max Acceleration", maxAccel)
            telemetry.update()
        }
    }

    companion object {
        @JvmField
        var TIME_TO_RUN = 1.5
    }
}