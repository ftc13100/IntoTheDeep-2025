package org.firstinspires.ftc.teamcode.legacy.auto.blue.park.left
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.OpMode
//import org.firstinspires.ftc.teamcode.constants.AutoStartPose
//import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
//
//@Autonomous(name = "Blue Park (Observation) Left", group = "Observation", preselectTeleOp = "MainTeleOp")
//class BlueParkObservationLeft : OpMode() {
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_LEFT.startPose)
//            .strafeRight(48.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.BLUE_LEFT.startPose
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}