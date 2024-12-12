package org.firstinspires.ftc.teamcode.legacy.auto.red.park.right

//@Autonomous(name = "Red Park (Submmersible) Right", group = "Submmersible", preselectTeleOp = "MainTeleOp")
//class RedParkSubmmersibleRight: OpMode() {
//
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_RIGHT.startPose)
//            .forward(27.0)
//            .strafeLeft(48.0)
//            .forward(24.0)
//            .strafeRight(12.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.RED_RIGHT.startPose
//
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}