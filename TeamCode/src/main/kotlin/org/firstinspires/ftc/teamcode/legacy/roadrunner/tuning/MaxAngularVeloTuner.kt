package org.firstinspires.ftc.teamcode.legacy.roadrunner.tuning
//
//import com.acmerobotics.dashboard.FtcDashboard
//import com.acmerobotics.dashboard.config.Config
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.hardware.DcMotor
//import com.qualcomm.robotcore.util.ElapsedTime
//import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
//
///**
// * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
// *
// *
// * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
// *
// *
// * Further fine tuning of MAX_ANG_VEL may be desired.
// */
////@Disabled
//@Config
//@Autonomous(group = "drive")
//class MaxAngularVeloTuner : LinearOpMode() {
//    private var timer: ElapsedTime? = null
//    private var maxAngVelocity = 0.0
//
//    @Throws(InterruptedException::class)
//    override fun runOpMode() {
//        val drive = DriveSubsystem(hardwareMap)
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
//        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.")
//        telemetry.addLine("Please ensure you have enough space cleared.")
//        telemetry.addLine("")
//        telemetry.addLine("Press start when ready.")
//        telemetry.update()
//
//        waitForStart()
//
//        telemetry.clearAll()
//        telemetry.update()
//
//        drive.setDrivePower(Pose2d(0.0, 0.0, 1.0))
//
//        timer = ElapsedTime()
//
//        while (!isStopRequested && timer!!.seconds() < RUNTIME) {
//            drive.updatePoseEstimate()
//
//            val (_, _, heading) = requireNotNull(
//                drive.poseVelocity
//            ) { "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer." }
//
//            maxAngVelocity = heading.coerceAtLeast(maxAngVelocity)
//        }
//        drive.setDrivePower(Pose2d())
//        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity)
//        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity))
//        telemetry.update()
//        while (!isStopRequested) idle()
//    }
//
//    companion object {
//        var RUNTIME = 4.0
//    }
//}