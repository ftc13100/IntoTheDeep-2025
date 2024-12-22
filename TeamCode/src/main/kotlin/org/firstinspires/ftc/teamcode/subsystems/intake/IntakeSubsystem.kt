package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard

object IntakeSubsystem : SubsystemBase() {
    private lateinit var claw: Servo
    private lateinit var wrist: CRServo
    private lateinit var intakeEncoder: Encoder

    val position
        get() = intakeEncoder.getPositionAndVelocity().position

    val velocity
        get() = intakeEncoder.getPositionAndVelocity().velocity

    @JvmField var kP = 0.0
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0

    private val controller = PIDController(kP, kI, kD)

    var target = controller.setPoint
        set(value) {
            controller.setPoint = value
            field = value
        }

    fun initialize(hardwareMap: HardwareMap) {
        claw = hardwareMap[Servo::class.java, ControlBoard.INTAKE.deviceName]
        wrist = CRServo(hardwareMap, ControlBoard.INTAKE_BELT.deviceName)

        intakeEncoder = OverflowEncoder(
            RawEncoder(
                hardwareMap[DcMotorEx::class.java, ControlBoard.INTAKE_ENCODER.deviceName]
            )
        )
    }

    fun closeClaw() {
        claw.position = 0.8
    }

    fun openClaw() {
        claw.position = 0.2
    }

    fun wristUp() {
        wrist.set(1.0)
    }

    fun wristDown() {
        wrist.set(-1.0)
    }

    fun wristStop() {
        wrist.stop()
    }

    fun operateWrist() {
        controller.setPID(kP, kI, kD)

        val output = controller.calculate(position.toDouble())

        wrist.set(output)
    }

    fun setClaw(position: Double) { claw.position = position }
}