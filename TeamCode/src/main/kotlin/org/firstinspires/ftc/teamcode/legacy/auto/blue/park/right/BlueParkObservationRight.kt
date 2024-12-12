package org.firstinspires.ftc.teamcode.legacy.auto.blue.park.right

//@Autonomous(name = "Blue Park (Observation) Right", group = "Observation", preselectTeleOp = "MainTeleOp")
//class BlueParkObservationRight: OpMode() {
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_RIGHT.startPose)
//            .strafeRight(72.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.BLUE_RIGHT.startPose
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}