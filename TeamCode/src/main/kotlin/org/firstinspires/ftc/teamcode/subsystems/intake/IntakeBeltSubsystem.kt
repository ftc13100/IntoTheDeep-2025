package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeBeltSubsystem(
    private val intakeBelt: Servo
) : SubsystemBase() {
    fun outtakePos() {
        intakeBelt.position += 0.01
    }

    fun intakePos() {
        intakeBelt.position -= 0.01
    }

    fun setPos(pos: Double) {
        intakeBelt.position = pos
    }
}