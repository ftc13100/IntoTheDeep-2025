package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

class ManualFeedbackTuner : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        when (TuningOpModes.DRIVE_CLASS) {
            DriveSubsystem::class.java -> {
                DriveSubsystem.initialize(hardwareMap)

                val drive = DriveSubsystem

                if (drive.localizer is ThreeDeadWheelLocalizer) {
                    if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0.0 && ThreeDeadWheelLocalizer.Companion.PARAMS.par1YTicks == 1.0) {
                        throw RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
                    }
                }
                waitForStart()

                while (opModeIsActive()) {
                    runBlocking(
                        drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                            .lineToX(DISTANCE)
                            .lineToX(0.0)
                            .build()
                    )
                }
            }

            else -> {
                throw RuntimeException()
            }
        }
    }

    companion object {
        var DISTANCE: Double = 64.0
    }
}
