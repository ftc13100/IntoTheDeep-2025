package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.IntakeConstants
import kotlin.math.PI
import kotlin.math.cos

@Config
object IntakeSubsystem : SubsystemBase() {
    private lateinit var claw: Servo
    private lateinit var wrist: CRServo

    private lateinit var intakeEncoder: Encoder
    private lateinit var intakeTouch: TouchSensor

    val isPressed: Boolean
        get() = intakeTouch.isPressed

    var position = Math.toRadians(90.0)
        get() = intakeEncoder.getPositionAndVelocity().position / 8192.0 * 2 * PI + Math.toRadians(90.0)
        private set

    val velocity
        get() = intakeEncoder.getPositionAndVelocity().velocity / 8192.0 * 2 * PI

    @JvmField var kP = 1.0
    @JvmField var kI = 0.0
    @JvmField var kD = 0.02
    @JvmField var kCos = 0.05

    private val controller = ProfiledPIDController(
        IntakeConstants.kP.value,
        IntakeConstants.kI.value,
        IntakeConstants.kD.value,
        TrapezoidProfile.Constraints(
            IntakeConstants.INTAKE_MAX_VELOCITY.value,
            IntakeConstants.INTAKE_MAX_ACCELERATION.value
        )
    )

    var target = controller.goal.position
        set(value) {
            controller.goal = TrapezoidProfile.State(value, 0.0)
            field = value
        }

    val isBusy
        get() = controller.atGoal()

    fun initialize(hardwareMap: HardwareMap) {
        claw = hardwareMap[Servo::class.java, ControlBoard.INTAKE.deviceName]
        wrist = CRServo(hardwareMap, ControlBoard.INTAKE_BELT.deviceName)
        intakeTouch = hardwareMap[TouchSensor::class.java, ControlBoard.INTAKE_TOUCH.deviceName]

        val intakeEnc = hardwareMap[DcMotorEx::class.java, ControlBoard.INTAKE_ENCODER.deviceName]
        intakeEnc.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeEnc.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        controller.setTolerance(Math.toRadians(3.0))

        intakeEncoder = OverflowEncoder(RawEncoder(intakeEnc))
    }

    fun closeClaw() {
        claw.position = 0.485
    }

    fun openClaw() {
        claw.position = 0.25
    }

    fun wristUp() = if (!isPressed) wrist.set(1.0) else wrist.stop()


    fun wristDown() = wrist.set(-1.0)


    fun wristStop() = wrist.set(kCos * cos(position))

    fun reset() {
        wrist.stop()
        position = Math.toRadians(90.0)
    }

    fun operateWrist() {
        controller.setPID(kP, kI, kD)
//        feedforward = ArmFeedforward(0.0, kCos, 0.0)

        val output = controller.calculate(position) +
                kCos * cos(position)

        wrist.set(output)
    }

    fun setClaw(position: Double) { claw.position = position }
}