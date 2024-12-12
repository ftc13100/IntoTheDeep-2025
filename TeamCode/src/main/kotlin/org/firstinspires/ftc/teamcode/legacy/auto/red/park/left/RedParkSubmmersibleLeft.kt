package org.firstinspires.ftc.teamcode.legacy.auto.red.park.left

//@Autonomous(name = "Red Park (Submmersible) Left", group = "Submmersible", preselectTeleOp = "MainTeleOp")
//class RedParkSubmmersibleLeft: OpMode() {
//
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_LEFT.startPose)
//            .forward(27.0)
//            .strafeLeft(24.0)
//            .forward(24.0)
//            .strafeRight(12.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.RED_LEFT.startPose
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}