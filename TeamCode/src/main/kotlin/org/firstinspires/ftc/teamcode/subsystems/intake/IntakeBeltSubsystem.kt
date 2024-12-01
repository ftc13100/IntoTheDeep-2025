package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeBeltSubsystem(
    private val intakeBelt: Servo
) : SubsystemBase() {
    var beltPos = false

    fun outtakePos() {
        intakeBelt.position = 0.75
        beltPos = !beltPos
    }

    fun intakePos() {
        intakeBelt.position = 0.5
        beltPos = !beltPos
    }

    fun increasePos() {
        intakeBelt.position += 0.005
    }

    fun decreasePos() {
        intakeBelt.position -= 0.005
    }

    fun setPos(pos: Double) {
        intakeBelt.position = pos
    }
}