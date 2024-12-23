package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import kotlin.math.PI

@Config
object IntakeSubsystem : SubsystemBase() {
    private lateinit var claw: Servo
    private lateinit var wrist: CRServo
    private lateinit var intakeEncoder: Encoder

    val position
        get() = intakeEncoder.getPositionAndVelocity().position / 8192.0 * 2 * PI

    val velocity
        get() = intakeEncoder.getPositionAndVelocity().velocity / 8192.0 * 2 * PI

    @JvmField var kP = 0.0
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0
    @JvmField var kCos = 0.0

    private val controller = PIDController(kP, kI, kD)
    private var feedforward = ArmFeedforward(0.0, kCos, 0.0)

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
        feedforward = ArmFeedforward(0.0, kCos, 0.0)

        val output = controller.calculate(position) + feedforward.calculate(position + ArmSubsystem.angle, velocity)

        wrist.set(output)
    }

    fun setClaw(position: Double) { claw.position = position }
}