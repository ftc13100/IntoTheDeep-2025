package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import kotlin.math.PI

@Config
class ArmSubsystem(
    armLeft: Motor,
    armRight: Motor,
    private val telemetry: Telemetry
) : PIDSubsystem(
    PIDController(
        ArmConstants.kP.value,
        ArmConstants.kI.value,
        ArmConstants.kD.value,
    )
) {
    private val turnMotors = MotorGroup(armLeft, armRight)

    val armAngle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_312.cpr * PI

    val armVelocity: Double
        get() = turnMotors.velocities[0] / GoBILDA.RPM_312.cpr * PI

    private var feedforward = ArmFeedforward(0.0, ArmConstants.kCos.value, 0.0);

    init {
        armLeft.inverted = true
        armLeft.encoder.setDirection(Motor.Direction.REVERSE)

        turnMotors.inverted = true

        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun useOutput(output: Double, setpoint: Double) {
        // For tuning only
        controller.setPIDF(kP, kI, kD, 0.0)
        feedforward = ArmFeedforward(0.0, kCos, 0.0)

        telemetry.addData("Output: ", output)
        telemetry.addData("FF output: ", feedforward.calculate(armAngle, armVelocity))
        telemetry.update()

        turnMotors.set(output + feedforward.calculate(armAngle, armVelocity))
    }

    override fun getMeasurement() = armAngle

    companion object {
        @JvmField
        var kCos = 0.4

        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0
    }
}