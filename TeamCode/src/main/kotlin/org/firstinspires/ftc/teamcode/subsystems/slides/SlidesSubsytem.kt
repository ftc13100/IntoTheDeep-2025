package org.firstinspires.ftc.teamcode.subsystems.slides

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.constants.SlidesConstants
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem

@Config
class SlidesSubsytem(
    elevatorRight : Motor,
    elevatorLeft : Motor
) : PIDSubsystem(
    PIDController(
        SlidesConstants.kP.value,
        SlidesConstants.kI.value,
        SlidesConstants.kD.value,
    )
) {
    private val extendMotors = MotorGroup(elevatorRight, elevatorLeft)

    val slidePos: Double
        get() = extendMotors.positions[0]

    val slideVelocity: Double
        get() = extendMotors.velocities[0]

    init {
        elevatorRight.inverted = true

        extendMotors.resetEncoder()
        extendMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun useOutput(output: Double, setpoint: Double) {
        extendMotors.set(output)
    }

    override fun getMeasurement() = slidePos
}