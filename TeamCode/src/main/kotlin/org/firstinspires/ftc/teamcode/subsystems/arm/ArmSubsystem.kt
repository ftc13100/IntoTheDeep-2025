package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.State
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import kotlin.math.PI
import kotlin.math.cos

@Config
object ArmSubsystem : SubsystemBase() {
    private lateinit var turnMotors: MotorGroup

    private const val ARM_MIN = 0.0
    private const val ARM_MAX = 1.5708

    val velocity: Double
        get() = turnMotors.velocities[0] / GoBILDA.RPM_30.cpr * 2 * PI

    val angle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_30.cpr * 2 * PI

    private var feedforward = ArmFeedforward(
        ArmConstants.kS.value,
        ArmConstants.kCos.value,
        ArmConstants.kV.value
    )

    private val controller = PIDController(
        ArmConstants.kP.value,
        ArmConstants.kI.value,
        ArmConstants.kD.value,
    )

    var target = 0.0

    val isBusy
        get() = controller.atSetPoint()

    var telemetry: Telemetry? = null

    var enabled = true

    fun initialize(hardwareMap: HardwareMap, telemetry: Telemetry? = null) {
        val armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        val armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        armLeft.inverted = true

        turnMotors = MotorGroup(armLeft, armRight)

        this.telemetry = telemetry

        turnMotors.resetEncoder()
        turnMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }


    fun clockwise() {
        turnMotors.set(1.0)
    }

    fun anticlockwise() {
        turnMotors.set(-1.0)
    }

    fun stop() {
        turnMotors.set(
            ArmConstants.kCos.value * cos(angle)
        )
    }

    fun operateArm(state: State) {
//        controller.setPID(kP, kI, kD)
//        feedforward = ArmFeedforward(kS, kCos, kV)

        val output = controller.calculate(angle, state.position) +
                feedforward.calculate(state.position, state.velocity)

        telemetry?.addData("Target Position", state.position)
        telemetry?.addData("Target Velocity", state.velocity)
        telemetry?.addData("Measured Position", angle)
        telemetry?.addData("Measured Velocity", velocity)

        telemetry?.update()

        if (enabled)
            turnMotors.set(output)
    }

    @JvmField var kP = 5.0
    @JvmField var kI = 0.0
    @JvmField var kD = 0.1
    @JvmField var kCos = 0.07
    @JvmField var kS = 0.002
    @JvmField var kV = 0.32
}