package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeBeltSubsystem(
    private val intakeBelt: Servo
) : SubsystemBase() {
    var intakePos = false

    fun outtakePos() {
        intakeBelt.position = 0.0
        intakePos = true
    }

    fun intakePos() {
        intakeBelt.position = 0.6
        intakePos = false
    }


    fun setPos(pos: Double) {
        intakeBelt.position = pos
    }
}