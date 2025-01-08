package org.firstinspires.ftc.teamcode.subsystems.drive


import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage

/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 *
 *
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
@Config
class PinpointDrive(
    hardwareMap: HardwareMap,
    pose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) : MecanumDrive(hardwareMap, pose) {
    val pinpoint: GoBildaPinpointDriverRR
    private var lastPinpointPose = pose

    init {
        write("PINPOINT_PARAMS", PARAMS)
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR::class.java, PARAMS.pinpointDeviceName)

        if (PARAMS.usePinpointIMUForTuning) {
            lazyImu = LazyImu(
                hardwareMap,
                PARAMS.pinpointDeviceName,
                RevHubOrientationOnRobot(RevHubOrientationOnRobot.zyxOrientation(0.0, 0.0, 90.0))
            )
        }

        // RR localizer note: don't love this conversion (change driver?)
        pinpoint.setOffsets(
            DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(
                PARAMS.yOffset
            )
        )


        pinpoint.setEncoderResolution(PARAMS.encoderResolution)

        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection)

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU()
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300)
        } catch (e: InterruptedException) {
            throw RuntimeException(e)
        }

        pinpoint.setPosition(pose)
    }

    class Params {
        /*
        Set this to the name that your Pinpoint is configured as in your hardware config.
         */
        var pinpointDeviceName = "pinpoint"

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of the center is a negative number. The Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is: forward of the center is a positive number,
        backwards is a negative number.
         */
        //These are tuned for 3110-0002-0001 Product Insight #1
        // RR localizer note: These units are inches, presets are converted from mm (which is why they are inexact)
        var xOffset = -3.3071
        var yOffset = -6.6142

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, input the number of ticks per millimeter for that pod.

        RR LOCALIZER NOTE: this is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; the value for the Gobilda 4 Bar Pod is approximately 20.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
        var encoderResolution = GoBildaPinpointDriverRR.goBILDA_SWINGARM_POD.toDouble()

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        var xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD
        var yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD

        /*
        Use the pinpoint IMU for tuning
        If true, overrides any IMU setting in MecanumDrive and uses exclusively Pinpoint for tuning
        You can also use the pinpoint directly in MecanumDrive if this doesn't work for some reason;
         replace "imu" with "pinpoint" or whatever your pinpoint is called in config.
         Note: Pinpoint IMU is always used for base localization
         */
        var usePinpointIMUForTuning = true
    }

    override fun updatePoseEstimate(): PoseVelocity2d {
        if (lastPinpointPose !== pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override the sensor's pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            pinpoint.setPosition(pose)
        }
        pinpoint.update()
        pose = pinpoint.positionRR
        lastPinpointPose = pose

        // RR standard
        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        write("ESTIMATED_POSE", PoseMessage(pose))
        write("PINPOINT_RAW_POSE", FTCPoseMessage(pinpoint.position))
        write("PINPOINT_STATUS", pinpoint.deviceStatus)

        return pinpoint.velocityRR
    }

    // for debug logging
    class FTCPoseMessage(pose: Pose2D) {
        var timestamp: Long = System.nanoTime()
        var x: Double = pose.getX(DistanceUnit.INCH)
        var y: Double = pose.getY(DistanceUnit.INCH)
        var heading: Double = pose.getHeading(AngleUnit.RADIANS)
    }

    companion object {
        @JvmField var PARAMS = Params()
    }
}