package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

class SplineTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        when (TuningOpModes.DRIVE_CLASS) {
            DriveSubsystem::class.java -> {
                DriveSubsystem.initialize(hardwareMap)

                val drive = DriveSubsystem

                waitForStart()

                runBlocking(
                    drive.actionBuilder(beginPose)
                        .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                        .splineTo(Vector2d(0.0, 60.0), Math.PI)
                        .build()
                )
            }

            else -> {
                throw RuntimeException()
            }
        }
    }
}
