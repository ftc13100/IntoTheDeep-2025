package org.firstinspires.ftc.teamcode.subsystems.slides

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.constants.SlidesConstants
import kotlin.math.sin

class OpenElevatorSubsystem(
    //sets them as a private variable thats a motor (same name as in driver hub)
    elevatorRight : Motor,
    elevatorLeft : Motor,
    private val slideAngle: () -> Double
) : SubsystemBase() {
    //makes a motor group bc you have to move them at the same time
    private val elevatorMotors = MotorGroup(elevatorRight, elevatorLeft)
    val slidePos: Double
        get() = elevatorMotors.positions[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    init {
        elevatorRight.inverted = true
        elevatorRight.encoder.setDirection(Motor.Direction.REVERSE)

        elevatorMotors.resetEncoder()
    }

    // Functions for moving slides up and down, number being speed, 1 being 100%
    fun up() {
        elevatorMotors.set(1.0)
    }

    fun down() {
        elevatorMotors.set(-1.0)
    }

    fun stop() {
        elevatorMotors.set(SlidesConstants.kG.value * sin(slideAngle.invoke()))
    }

}