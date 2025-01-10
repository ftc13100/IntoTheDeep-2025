package org.firstinspires.ftc.teamcode.legacy.auto.blue.park.left

//@Autonomous(name = "Blue Park (Submmersible) Left", group = "Submmersible", preselectTeleOp = "MainTeleOp")
//class BlueParkSubmmersibleLeft: OpMode() {
//
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_LEFT.startPose)
//            .forward(25.0)
//            .strafeLeft(48.0)
//            .forward(25.0)
//            .strafeRight(12.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.BLUE_LEFT.startPose
//
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}