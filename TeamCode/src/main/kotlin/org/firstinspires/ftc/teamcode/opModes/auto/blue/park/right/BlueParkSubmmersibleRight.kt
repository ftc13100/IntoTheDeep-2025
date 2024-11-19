package org.firstinspires.ftc.teamcode.opModes.auto.blue.park.right

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

@Autonomous(name = "Blue Park (Submmersible) Right", group = "Submmersible", preselectTeleOp = "MainTeleOp")
class BlueParkSubmmersibleRight: OpMode() {

    private lateinit var driveSubsystem: DriveSubsystem

    override fun init() {
        driveSubsystem = DriveSubsystem(hardwareMap)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_RIGHT.startPose)
            .forward(27.0)
            .strafeLeft(24.0)
            .forward(24.0)
            .strafeRight(12.0)
            .build()

        driveSubsystem.followTrajectorySequenceAsync(trajectory)

    }

    override fun loop() {
        driveSubsystem.update()
    }
}