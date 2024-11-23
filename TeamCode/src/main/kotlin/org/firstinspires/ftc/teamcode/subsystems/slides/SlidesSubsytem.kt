package org.firstinspires.ftc.teamcode.subsystems.slides

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import kotlin.math.PI

@Config
class SlidesSubsytem(
    elevatorLeft : Motor,
    elevatorRight : Motor
) : PIDSubsystem(
    PIDController(
        0.0,
        0.0,
        0.0,
    )
) {
    private val extendMotors = MotorGroup(elevatorLeft, elevatorRight)

    val slidePos: Double
        get() = extendMotors.positions[0]

    val slideVelocity: Double
        get() = extendMotors.velocities[0] / GoBILDA.RPM_435.cpr * 2 * PI

    init {
        elevatorLeft.inverted = true
        elevatorLeft.encoder.setDirection(Motor.Direction.REVERSE)

        extendMotors.resetEncoder()
        extendMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun useOutput(output: Double, setpoint: Double) {
        // For tuning only
        controller.setPIDF(kP, kI, kD, 0.0)

        extendMotors.set(output)
    }

    override fun getMeasurement() = slidePos

    companion object {

        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0
    }
}