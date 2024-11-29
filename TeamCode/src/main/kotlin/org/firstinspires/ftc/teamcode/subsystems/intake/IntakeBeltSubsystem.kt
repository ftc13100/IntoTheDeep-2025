package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeBeltSubsystem(
    private val intakeBelt: Servo
) : SubsystemBase() {
    fun intakePos() {
     intakeBelt.position = 0.0
    }
    fun outtakePos() {
        intakeBelt.position = 0.6
    }

    fun setPos(pos: Double) {
        intakeBelt.position = pos
    }
}