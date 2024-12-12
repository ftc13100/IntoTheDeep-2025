package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TankDrive

class SplineTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                val drive = MecanumDrive(hardwareMap, beginPose)

                waitForStart()

                runBlocking(
                    drive.actionBuilder(beginPose)
                        .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                        .splineTo(Vector2d(0.0, 60.0), Math.PI)
                        .build()
                )
            }
            TankDrive::class.java -> {
                val drive = TankDrive(hardwareMap, beginPose)

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
