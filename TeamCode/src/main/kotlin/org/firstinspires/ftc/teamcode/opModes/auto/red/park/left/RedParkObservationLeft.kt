package org.firstinspires.ftc.teamcode.opModes.auto.red.park.left

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

@Autonomous(name = "Red Park (Observation) Left", group = "Observation", preselectTeleOp = "MainTeleOp")
class RedParkObservationLeft: OpMode() {

    private lateinit var driveSubsystem: DriveSubsystem

    override fun init() {
        driveSubsystem = DriveSubsystem(hardwareMap)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_LEFT.startPose)
            .strafeRight(72.0)
            .build()

        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.update()
    }
}