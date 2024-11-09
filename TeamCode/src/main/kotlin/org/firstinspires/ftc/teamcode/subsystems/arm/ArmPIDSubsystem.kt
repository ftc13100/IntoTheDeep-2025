package org.firstinspires.ftc.teamcode.subsystems.arm

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem

class ArmPIDSubsystem(
    armLeft: Motor,
    armRight: Motor
) : PIDSubsystem(
    PIDController(
        0.0,
        0.0,
        0.0,
    )
) {
    private val turnMotors = MotorGroup(armLeft, armRight)

    init {
        armLeft.inverted = true
        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun useOutput(output: Double, setpoint: Double) {
        TODO("Not yet implemented")
    }

    override fun getMeasurement(): Double {
        TODO("Not yet implemented")
    }

}