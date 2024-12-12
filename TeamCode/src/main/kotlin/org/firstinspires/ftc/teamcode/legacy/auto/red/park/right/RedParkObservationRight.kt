package org.firstinspires.ftc.teamcode.legacy.auto.red.park.right

//@Autonomous(name = "Red Park (Observation) Right", group = "Observation", preselectTeleOp = "MainTeleOp")
//class RedParkObservationRight: OpMode() {
//
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_RIGHT.startPose)
//            .strafeRight(48.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.RED_RIGHT.startPose
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}