package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo

class IntakeSubsystem(
    private val intake: Servo
) : SubsystemBase() {
    fun intake() {
        intake.position = 1.0
    }

    fun outtake() {
        intake.position = 0.0
    }

    fun setSpeed(speed: Double) { intake.position = speed }
}