package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.CRServo

class IntakeSubsystem(
    private val intake: CRServo
) : SubsystemBase() {
    fun intake() = intake.set(1.0)

    fun outtake() = intake.set(-1.0)

    fun stop() = intake.stop()
}