package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import kotlin.math.PI
import kotlin.math.cos

@Config
object OpenArmSubsystem : SubsystemBase() {
    private lateinit var turnMotors: MotorGroup

    val armAngle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_312.cpr * 2 * PI

    fun initialize(hardwareMap: HardwareMap) {
        val armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        val armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        armLeft.inverted = true

        turnMotors = MotorGroup(armRight, armLeft)

        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }


// Here are functions that work the motor, and the double is the speed of the motor, 1 being 100%.
    fun clockwise() {
        turnMotors.set(0.50)
    }

    fun anticlockwise() {
        turnMotors.set(-0.50)
    }

    fun stop() {
        turnMotors.set(ArmConstants.kCos.value * cos(armAngle))
    }

}