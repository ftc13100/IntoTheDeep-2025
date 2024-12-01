package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import kotlin.math.PI

@Config
class ArmSubsystem(
    armRight: Motor,
    armLeft: Motor,
) : PIDSubsystem(
    PIDController(
        ArmConstants.kP.value,
        ArmConstants.kI.value,
        ArmConstants.kD.value
    )
) {
    private val turnMotors = MotorGroup(armRight, armLeft)

    val armAngle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_60.cpr * PI

    val armVelocity: Double
        get() = turnMotors.velocities[0] / GoBILDA.RPM_60.cpr * PI

    private var feedforward = ArmFeedforward(0.0, ArmConstants.kCos.value, 0.0);

    init {
        armLeft.inverted = true

        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun useOutput(output: Double, setpoint: Double) {
//        controller.setPIDF(kP, kI, kD, 0.0)
//        feedforward = ArmFeedforward(0.0, kCos, 0.0)

        turnMotors.set(output + feedforward.calculate(armAngle, armVelocity))
    }

    override fun getMeasurement() = armAngle

    companion object {
        @JvmField
        var kP = 2.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var kCos = 0.1

    }
}