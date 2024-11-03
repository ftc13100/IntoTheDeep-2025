package org.firstinspires.ftc.teamcode.subsystems.arm

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class ArmSubsystem(
// Here I am just declaring the motors and what they are called on our driver hub.
        private val armRight : Motor,
        private val armLeft : Motor,

) : SubsystemBase() {

//Here I am making a motor group, as the arm motors are going to work together to to turn the slides.

    private val turnMotors = MotorGroup(armRight, armLeft)

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
        turnMotors.set(0.0)
    }


}