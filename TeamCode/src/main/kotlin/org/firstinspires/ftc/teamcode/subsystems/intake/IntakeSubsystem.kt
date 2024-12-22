package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard

object IntakeSubsystem : SubsystemBase() {
    private lateinit var intake: Servo
    private lateinit var intakeBelt: Servo
    private lateinit var intakeEncoder: Encoder

    val data
        get() = intakeEncoder.getPositionAndVelocity()

    fun initialize(hardwareMap: HardwareMap) {
        intake = hardwareMap[Servo::class.java, ControlBoard.INTAKE.deviceName]
        intakeBelt = hardwareMap[Servo::class.java, ControlBoard.INTAKE_BELT.deviceName]

        intakeEncoder = OverflowEncoder(
            RawEncoder(
                hardwareMap[DcMotorEx::class.java, ControlBoard.INTAKE_ENCODER.deviceName]
            )
        )
    }

    fun intake() {
        intake.position = 0.8
    }

    fun outtake() {
        intake.position = 0.2
    }

    fun outtakePos() {
        intakeBelt.position = 0.4
    }

    fun intakePos() {
        intakeBelt.position = 0.0
    }

    fun increasePos() {
        intakeBelt.position += 0.005
    }

    fun decreasePos() {
        intakeBelt.position -= 0.005
    }

    fun setPos(pos: Double) {
        intakeBelt.position = pos
    }

    fun setSpeed(speed: Double) { intake.position = speed }
}