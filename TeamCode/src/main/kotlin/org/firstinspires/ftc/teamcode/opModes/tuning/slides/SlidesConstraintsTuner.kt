package org.firstinspires.ftc.teamcode.opModes.tuning.slides

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous
class SlidesConstraintsTuner : LinearOpMode() {
    private lateinit var leftMotor: Motor
    private lateinit var rightMotor: Motor

    private lateinit var elevator: ElevatorSubsystem

    private var maxVel = 0.0
    private var maxAccel = 0.0

    private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)
    override fun runOpMode() {
        leftMotor = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        rightMotor = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        elevator = ElevatorSubsystem(rightMotor, leftMotor)
        waitForStart()

        var dt = 0L
        var dv = 0.0

        timer.reset()
        while (opModeIsActive()) {
            while (timer.seconds() < TIME_TO_RUN) {
                elevator.spinUp()

                dt = timer.nanoseconds() - dt
                dv = elevator.slideVelocity - dv / dt

                maxVel = elevator.slideVelocity.coerceAtLeast(maxVel)
                maxAccel = dv.coerceAtLeast(maxAccel)

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
        var TIME_TO_RUN = 2.0
    }
}