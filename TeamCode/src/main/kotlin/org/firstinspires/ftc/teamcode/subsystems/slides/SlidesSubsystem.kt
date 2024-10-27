package org.firstinspires.ftc.teamcode.subsystems.slides

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup

class SlidesSubsystem(

        //sets them as a private variable thats a motor (same name as in driver hub)
        private val leftSlideString : Motor,
        private val rightSlideString : Motor,

) : SubsystemBase() {
    //makes a motor group bc you have to move them at the same time
    private val elevatorMotors = MotorGroup(leftSlideString, rightSlideString)


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