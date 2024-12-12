package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage

@Config
class ThreeDeadWheelLocalizer(hardwareMap: HardwareMap, val inPerTick: Double) : Localizer {
    class Params {
        var par0YTicks: Double = 0.0 // y position of the first parallel encoder (in tick units)
        var par1YTicks: Double = 1.0 // y position of the second parallel encoder (in tick units)
        var perpXTicks: Double = 0.0 // x position of the perpendicular encoder (in tick units)
    }

    // TODO: make sure your config has **motors** with these names (or change them)
    //   the encoders should be plugged into the slot matching the named motor
    //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
    val par0: Encoder =
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "par0")))
    val par1: Encoder =
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "par1")))
    val perp: Encoder =
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "perp")))

    // TODO: reverse encoder directions if needed
    //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

    private var lastPar0Pos = 0
    private var lastPar1Pos = 0
    private var lastPerpPos = 0
    private var initialized = false

    init {
        write("THREE_DEAD_WHEEL_PARAMS", PARAMS)
    }

    override fun update(): Twist2dDual<Time> {
        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()

        write(
            "THREE_DEAD_WHEEL_INPUTS",
            ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel)
        )

        if (!initialized) {
            initialized = true

            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            lastPerpPos = perpPosVel.position

            return Twist2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            )
        }

        val par0PosDelta = par0PosVel.position - lastPar0Pos
        val par1PosDelta = par1PosVel.position - lastPar1Pos
        val perpPosDelta = perpPosVel.position - lastPerpPos

        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    doubleArrayOf(
                        (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    )
                ).times(inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                        (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                    )
                ).times(inPerTick)
            ),
            DualNum(
                doubleArrayOf(
                    (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                )
            )
        )

        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
        lastPerpPos = perpPosVel.position

        return twist
    }

    companion object {
        var PARAMS: Params = Params()
    }
}
