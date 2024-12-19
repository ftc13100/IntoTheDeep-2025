package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.roadrunner.messages.ThreeDeadWheelInputsMessage

@Config
class ThreeDeadWheelLocalizer(hardwareMap: HardwareMap, val inPerTick: Double) : Localizer {
    data class Params(
        @JvmField var leftYTicks: Double = -2547.028032399116, // y position of the first parallel encoder (in tick units)
        @JvmField var rightYTicks: Double = 2556.586233603863, // y position of the second parallel encoder (in tick units)
        @JvmField var strafeXTicks: Double = -490.4056033307692 // x position of the perpendicular encoder (in tick units)
    )

    // TODO: make sure your config has **motors** with these names (or change them)
    //   the encoders should be plugged into the slot matching the named motor
    //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
    val left =
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, ControlBoard.ODO_LEFT_ENCODER.deviceName)))
    val right =
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, ControlBoard.ODO_RIGHT_ENCODER.deviceName)))
    val strafe =
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, ControlBoard.ODO_STRAFE_ENCODER.deviceName)))

    // TODO: reverse encoder directions if needed
    //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

    private var lastLeftPos = 0
    private var lastRightPos = 0
    private var lastStrafePos = 0
    private var initialized = false

    init {
        write("THREE_DEAD_WHEEL_PARAMS", PARAMS)
    }

    override fun update(): Twist2dDual<Time> {
        val leftPosVel = left.getPositionAndVelocity()
        val rightPosVel = right.getPositionAndVelocity()
        val strafePosVel = strafe.getPositionAndVelocity()

        write(
            "THREE_DEAD_WHEEL_INPUTS",
            ThreeDeadWheelInputsMessage(leftPosVel, rightPosVel, strafePosVel)
        )

        if (!initialized) {
            initialized = true

            lastLeftPos = leftPosVel.position
            lastRightPos = rightPosVel.position
            lastStrafePos = strafePosVel.position

            return Twist2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            )
        }

        val leftPosDelta = leftPosVel.position - lastLeftPos
        val rightPosDelta = rightPosVel.position - lastRightPos
        val strafePosDelta = strafePosVel.position - lastStrafePos

        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    doubleArrayOf(
                        (PARAMS.leftYTicks * rightPosDelta - PARAMS.rightYTicks * leftPosDelta) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                        (PARAMS.leftYTicks * rightPosVel.velocity - PARAMS.rightYTicks * leftPosVel.velocity) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                    )
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        (PARAMS.strafeXTicks / (PARAMS.leftYTicks - PARAMS.rightYTicks) * (rightPosDelta - leftPosDelta) + strafePosDelta),
                        (PARAMS.strafeXTicks / (PARAMS.leftYTicks - PARAMS.rightYTicks) * (rightPosVel.velocity - leftPosVel.velocity) + strafePosVel.velocity),
                    )
                ) * inPerTick
            ),
            DualNum(
                doubleArrayOf(
                    (leftPosDelta - rightPosDelta) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                    (leftPosVel.velocity - rightPosVel.velocity) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                )
            )
        )

        lastLeftPos = leftPosVel.position
        lastRightPos = rightPosVel.position
        lastStrafePos = strafePosVel.position

        return twist
    }

    companion object {
        @JvmField var PARAMS = Params()
    }
}
