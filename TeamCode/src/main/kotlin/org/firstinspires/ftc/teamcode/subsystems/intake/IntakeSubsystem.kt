package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeSubsystem(
    private val intake: Servo
) : SubsystemBase() {
    var intakePos = false

    fun intake() {
        intake.position = 1.0
        intakePos = !intakePos
    }

    fun outtake() {
        intake.position = 0.6
        intakePos = !intakePos
    }

    fun setSpeed(speed: Double) { intake.position = speed }
}