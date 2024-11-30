package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeSubsystem(
    private val intake: Servo
) : SubsystemBase() {
    var intakePos = false

    fun intake() {
        intake.position = 0.6
        intakePos = true
    }

    fun outtake() {
        intake.position = 0.0
        intakePos = false
    }

    fun setSpeed(speed: Double) { intake.position = speed }
}