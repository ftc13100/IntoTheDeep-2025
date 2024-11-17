package org.firstinspires.ftc.teamcode.utils.roadrunner.util

import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.sign

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction
 */
class Encoder @JvmOverloads constructor(
    private val motor: DcMotorEx,
    private val clock: NanoClock = NanoClock.system(),
) {
    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     *
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    var direction: Direction
    private var lastPosition = 0
    private var velocityEstimate = 0.0
    private var lastUpdateTime: Double

    init {
        direction = Direction.FORWARD
        lastUpdateTime = clock.seconds()
    }

    private val multiplier: Int
        get() = direction.multiplier * if (motor.direction == DcMotorSimple.Direction.FORWARD) 1 else -1
    val currentPosition: Int
        get() {
            val multiplier = multiplier
            val currentPosition = motor.currentPosition * multiplier
            if (currentPosition != lastPosition) {
                val currentTime = clock.seconds()
                val dt = currentTime - lastUpdateTime
                velocityEstimate = (currentPosition - lastPosition) / dt
                lastPosition = currentPosition
                lastUpdateTime = currentTime
            }
            return currentPosition
        }
    private val rawVelocity: Double
        get() {
            val multiplier = multiplier
            return motor.velocity * multiplier
        }
    val correctedVelocity: Double
        get() = inverseOverflow(
            rawVelocity, velocityEstimate
        )

    enum class Direction(val multiplier: Int) {
        FORWARD(1), REVERSE(-1)

    }

    companion object {
        private const val CPS_STEP = 0x10000
        private fun inverseOverflow(input: Double, estimate: Double): Double {
            var real = input
            while (abs(estimate - real) > CPS_STEP / 2.0) {
                real += sign(estimate - real) * CPS_STEP
            }
            return real
        }
    }
}