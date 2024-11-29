package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeBeltSubsytem(
    private val intakeBelt: Servo
) : SubsystemBase() {
    fun intake() { intakeBelt.position = 0.5 }

    fun outtake() { intakeBelt.position = 0.0 }
}