package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import kotlin.math.PI

@Config
class OpenArmSubsystem(
// Here I am just declaring the motors and what they are called on our driver hub.
        armRight : Motor,
        armLeft : Motor,

) : SubsystemBase() {

//Here I am making a motor group, as the arm motors are going to work together to to turn the slides.

    private val turnMotors = MotorGroup(armRight, armLeft)
    val armAngle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_435.cpr * 2 * PI

    init {
        armLeft.inverted = true
        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }


// Here are functions that work the motor, and the double is the speed of the motor, 1 being 100%.
    fun clockwise() {
        turnMotors.set(0.35)
    }

    fun anticlockwise() {
        turnMotors.set(-0.35)
    }

    fun stop() {
        turnMotors.set(kCos * Math.cos(armAngle))
    }

    companion object {
        @JvmField
        var kCos = 0.3
    }
}