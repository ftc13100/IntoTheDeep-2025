package org.firstinspires.ftc.teamcode.legacy.roadrunner.tuning

/**
 * This routine determines the effective forward offset for the lateral tracking wheel.
 * The procedure executes a point turn at a given angle for a certain number of trials,
 * along with a specified delay in milliseconds. The purpose of this is to track the
 * change in the y position during the turn. The offset, or distance, of the lateral tracking
 * wheel from the center or rotation allows the wheel to spin during a point turn, leading
 * to an incorrect measurement for the y position. This creates an arc around around
 * the center of rotation with an arc length of change in y and a radius equal to the forward
 * offset. We can compute this offset by calculating (change in y position) / (change in heading)
 * which returns the radius if the angle (change in heading) is in radians. This is based
 * on the arc length formula of length = theta * radius.
 *
 *
 * To run this routine, simply adjust the desired angle and specify the number of trials
 * and the desired delay. Then, run the procedure. Once it finishes, it will print the
 * average of all the calculated forward offsets derived from the calculation. This calculated
 * forward offset is then added onto the current forward offset to produce an overall estimate
 * for the forward offset. You can run this procedure as many times as necessary until a
 * satisfactory result is produced.
 */
//@Disabled
//@Config
//@Autonomous(group = "drive")
//class TrackingWheelForwardOffsetTuner : LinearOpMode() {
//    @Throws(InterruptedException::class)
//    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
//        val drive = DriveSubsystem(hardwareMap)
//        if (drive.localizer !is StandardTrackingWheelLocalizer) {
//            RobotLog.setGlobalErrorMsg(
//                "StandardTrackingWheelLocalizer is not being set in the "
//                        + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
//                        + "(hardwareMap));\" is called in SampleMecanumDrive.java"
//            )
//        }
//        telemetry.addLine("Press play to begin the forward offset tuner")
//        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly")
//        telemetry.update()
//        waitForStart()
//        if (isStopRequested) return
//        telemetry.clearAll()
//        telemetry.addLine("Running...")
//        telemetry.update()
//        val forwardOffsetStats = MovingStatistics(NUM_TRIALS)
//        for (i in 0 until NUM_TRIALS) {
//            drive.poseEstimate = Pose2d()
//
//            // it is important to handle heading wraparounds
//            var headingAccumulator = 0.0
//            var lastHeading = 0.0
//            drive.turnAsync(Math.toRadians(ANGLE))
//            while (!isStopRequested && drive.isBusy) {
//                val heading = drive.poseEstimate.heading
//                headingAccumulator += norm(heading - lastHeading)
//                lastHeading = heading
//                drive.update()
//            }
//            val forwardOffset = StandardTrackingWheelLocalizer.FORWARD_OFFSET +
//                    drive.poseEstimate.y / headingAccumulator
//            forwardOffsetStats.add(forwardOffset)
//            sleep(DELAY.toLong())
//        }
//        telemetry.clearAll()
//        telemetry.addLine("Tuning complete")
//        telemetry.addLine(
//            Misc.formatInvariant(
//                "Effective forward offset = %.2f (SE = %.3f)",
//                forwardOffsetStats.mean,
//                forwardOffsetStats.standardDeviation / Math.sqrt(NUM_TRIALS.toDouble())
//            )
//        )
//        telemetry.update()
//        while (!isStopRequested) {
//            idle()
//        }
//    }
//
//    companion object {
//        var ANGLE = 180.0 // deg
//        var NUM_TRIALS = 5
//        var DELAY = 1000 // ms
//    }
//}