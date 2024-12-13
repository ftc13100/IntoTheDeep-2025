package org.firstinspires.ftc.teamcode.subsystems.slides

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.SlidesConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.OpenArmSubsystem
import kotlin.math.sin

object OpenSlidesSubsystem: SubsystemBase() {
    private lateinit var elevatorMotors: MotorGroup

    val slidePos: Double
        get() = elevatorMotors.positions[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    val slideVelocity: Double
        get() = -elevatorMotors.velocities[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    fun initialize(hardwareMap: HardwareMap) {
        val elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        val elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)

        elevatorLeft.inverted = true

        elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)
    }

    fun up() {
        elevatorMotors.set(1.0)
    }

    fun down() {
        elevatorMotors.set(-1.0)
    }

    fun stop() {
        elevatorMotors.set(SlidesConstants.kG.value * sin(OpenArmSubsystem.armAngle))
    }

}