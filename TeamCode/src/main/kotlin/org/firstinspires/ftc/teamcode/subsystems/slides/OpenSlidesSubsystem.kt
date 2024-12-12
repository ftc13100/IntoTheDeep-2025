package org.firstinspires.ftc.teamcode.subsystems.slides

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard

object OpenSlidesSubsystem: SubsystemBase() {
    private lateinit var elevatorMotors: MotorGroup

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
        elevatorMotors.set(0.0)
    }

}