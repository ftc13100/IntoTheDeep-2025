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

    val armAngle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_60.cpr * PI

    val armVelocity: Double
        get() = turnMotors.velocities[0] / GoBILDA.RPM_60.cpr * PI

    private val feedforward = ArmFeedforward(0.0, kCos, 0.0);

    init {
        armLeft.inverted = true
        armLeft.encoder.setDirection(Motor.Direction.REVERSE)

        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun useOutput(output: Double, setpoint: Double) {
        // For tuning only
        controller.setPIDF(kP, kI, kD, 0.0)

        turnMotors.set(output + feedforward.calculate(armAngle, armVelocity))
    }

    override fun getMeasurement() = armAngle

    companion object {
        @JvmField
        var kCos = 0.004

        @JvmField
        var kP = 1.0

        @JvmField
        var kI = 0.0001

        @JvmField
        var kD = 0.08
    }
}