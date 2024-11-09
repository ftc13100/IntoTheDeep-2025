package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import kotlin.math.PI

@Config
class ArmSubsystem(
// Here I am just declaring the motors and what they are called on our driver hub.
    armRight: Motor,
    armLeft: Motor,
    openLoop: Boolean
) : PIDSubsystem(
    PIDController(
        0.0,
        0.0,
        0.0
    )
) {

    //Here I am making a motor group, as the arm motors are going to work together to to turn the slides.
    private val feedforward = ArmFeedforward(0.0, kCos, 0.0)
    private val turnMotors = MotorGroup(armRight, armLeft)
    val armAngle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_435.cpr * 2 * PI

    init {
        armLeft.inverted = true
        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        if (openLoop)
            disable()
        else
            enable()
    }

    override fun useOutput(output: Double, setpoint: Double) {
        turnMotors.set(output + feedforward.calculate(getMeasurement(), 0.0))
    }

    override fun getMeasurement(): Double = armAngle

    // Here are functions that work the motor, and the double is the speed of the motor, 1 being 100%.
    fun clockwise() {
        turnMotors.set(0.35)
    }

    fun anticlockwise() {
        turnMotors.set(-0.35)
    }

    fun stop() {
        turnMotors.set(feedforward.calculate(getMeasurement(), 0.0))
    }

    companion object {
        @JvmField
        var kCos = 0.3
    }
}