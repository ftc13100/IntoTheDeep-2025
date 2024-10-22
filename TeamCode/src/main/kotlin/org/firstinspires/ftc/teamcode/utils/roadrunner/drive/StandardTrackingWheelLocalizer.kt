package org.firstinspires.ftc.teamcode.utils.roadrunner.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.utils.roadrunner.util.Encoder

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class StandardTrackingWheelLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(
    listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
        Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
    )
) {
    private val leftEncoder: Encoder
    private val rightEncoder: Encoder
    private val strafeEncoder: Encoder

    init {
        leftEncoder = Encoder(
            hardwareMap.get(
                DcMotorEx::class.java, ControlBoard.ODO_LEFT_ENCODER.deviceName
            )
        )
        rightEncoder = Encoder(
            hardwareMap.get(
                DcMotorEx::class.java, ControlBoard.ODO_RIGHT_ENCODER.deviceName
            )
        )
        strafeEncoder = Encoder(
            hardwareMap.get(
                DcMotorEx::class.java, ControlBoard.ODO_STRAFE_ENCODER.deviceName
            )
        )
    }

    override fun getWheelPositions() = listOf(
        encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
        encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
        encoderTicksToInches(strafeEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
    )

    override fun getWheelVelocities() = listOf(
        encoderTicksToInches(leftEncoder.correctedVelocity) * X_MULTIPLIER,
        encoderTicksToInches(rightEncoder.correctedVelocity) * X_MULTIPLIER,
        encoderTicksToInches(strafeEncoder.correctedVelocity) * Y_MULTIPLIER
    )

    companion object {
        val TICKS_PER_REV = 8192.0
        val WHEEL_RADIUS = 0.74803 // in
        val GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed

        @JvmField
        var LATERAL_DISTANCE = 15.48986087647907 // in; distance between the left and right wheels

        @JvmField
        var FORWARD_OFFSET = 0.0 // in; offset of the lateral wheel

        var X_MULTIPLIER = 0.9943625369724665 // Multiplier in the X direction
        var Y_MULTIPLIER = 0.9973390034543611 // Multiplier in the Y direction
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}