package org.firstinspires.ftc.teamcode.legacy.roadrunner

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Disabled
//@Config
//@Autonomous(group = "drive")
//class StraightTest : LinearOpMode() {
//    @Throws(InterruptedException::class)
//    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
//        val drive = DriveSubsystem(hardwareMap)
//        val trajectory = drive.trajectoryBuilder(Pose2d())
//            .forward(DISTANCE)
//            .build()
//        waitForStart()
//        if (isStopRequested) return
//        drive.followTrajectory(trajectory)
//        val (x, y, heading) = drive.poseEstimate
//        telemetry.addData("finalX", x)
//        telemetry.addData("finalY", y)
//        telemetry.addData("finalHeading", heading)
//        telemetry.update()
//        while (!isStopRequested && opModeIsActive());
//    }
//
//    companion object {
//        @JvmField
//        var DISTANCE = 60.0 // in
//    }
//}