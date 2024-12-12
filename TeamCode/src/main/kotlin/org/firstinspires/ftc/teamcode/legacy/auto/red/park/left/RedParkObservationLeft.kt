package org.firstinspires.ftc.teamcode.legacy.auto.red.park.left

//@Autonomous(name = "Red Park (Observation) Left", group = "Observation", preselectTeleOp = "MainTeleOp")
//class RedParkObservationLeft: OpMode() {
//
//    private lateinit var driveSubsystem: DriveSubsystem
//
//    override fun init() {
//        driveSubsystem = DriveSubsystem(hardwareMap)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_LEFT.startPose)
//            .strafeRight(72.0)
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.RED_LEFT.startPose
//
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}