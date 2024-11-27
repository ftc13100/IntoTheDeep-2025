package org.firstinspires.ftc.teamcode.subsystems.slides

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class OpenSlidesSubsystem(
    //sets them as a private variable thats a motor (same name as in driver hub)
    elevatorLeft: Motor,
    elevatorRight: Motor,
) : SubsystemBase() {
    //makes a motor group bc you have to move them at the same time
    private val elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)

    init {
        elevatorLeft.inverted = true
    }

    // Functions for moving slides up and down, number being speed, 1 being 100%
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