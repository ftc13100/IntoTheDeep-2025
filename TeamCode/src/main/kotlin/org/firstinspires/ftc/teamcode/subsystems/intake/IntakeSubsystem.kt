package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard

object IntakeSubsystem : SubsystemBase() {
    private lateinit var intake: Servo

    fun initialize(hardwareMap: HardwareMap) {
        intake = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE.deviceName)
    }

    var intakePos = false

    fun intake() {
        intake.position = 0.8
        intakePos = !intakePos
    }

    fun outtake() {
        intake.position = 0.2
        intakePos = !intakePos
    }

    fun setSpeed(speed: Double) { intake.position = speed }
}