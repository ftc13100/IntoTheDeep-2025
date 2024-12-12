package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard

object IntakeSubsystem : SubsystemBase() {
    private lateinit var intake: CRServo

    fun initialize(hardwareMap: HardwareMap) {
        intake = CRServo(hardwareMap, ControlBoard.INTAKE.deviceName)
    }

    fun intake() = intake.set(1.0)

    fun outtake() = intake.set(-0.5)

    fun stop() = intake.stop()
}