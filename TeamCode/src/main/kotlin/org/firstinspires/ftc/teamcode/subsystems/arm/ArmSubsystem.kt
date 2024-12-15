package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import kotlin.math.PI

@Config
object ArmSubsystem : PIDSubsystem(
    PIDController(
        ArmConstants.kP.value,
        ArmConstants.kI.value,
        ArmConstants.kD.value
    )
) {
    private lateinit var turnMotors: MotorGroup
    private var feedforward = ArmFeedforward(0.0, ArmConstants.kCos.value, 0.0);

    val velocity: Double
        get() = turnMotors.velocities[0] / GoBILDA.RPM_60.cpr * PI

    val angle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_60.cpr * PI

    fun initialize(hardwareMap: HardwareMap) : ArmSubsystem {
        val armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        val armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        armLeft.inverted = true

        turnMotors = MotorGroup(armRight, armLeft)

        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        disable()

        return this
    }


// Here are functions that work the motor, and the double is the speed of the motor, 1 being 100%.
    fun clockwise() {
        turnMotors.set(1.0)
    }

    fun anticlockwise() {
        turnMotors.set(-1.0)
    }

    fun stop() {
        turnMotors.set(0.0)
    }

    override fun useOutput(output: Double, setpoint: Double) {
//        controller.setPIDF(kP, kI, kD, 0.0)
//        feedforward = ArmFeedforward(0.0, kCos, 0.0)

        turnMotors.set(output + feedforward.calculate(angle, velocity))
    }

    override fun getMeasurement() = angle
}