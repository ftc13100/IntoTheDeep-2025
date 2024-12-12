package org.firstinspires.ftc.teamcode.legacy.auto.blue.park.right

//@Autonomous(name = "Blue Park (Submmersible) Right", group = "Submmersible", preselectTeleOp = "MainTeleOp")
//class BlueParkSubmmersibleRight: OpMode() {
//
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_RIGHT.startPose)
//            .forward(27.0)
//            .strafeLeft(24.0)
//            .forward(24.0)
//            .strafeRight(12.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.BLUE_RIGHT.startPose
//
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}