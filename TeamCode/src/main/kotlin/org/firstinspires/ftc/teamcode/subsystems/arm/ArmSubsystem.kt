package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import kotlin.math.PI
import kotlin.math.cos

@Config
object ArmSubsystem : SubsystemBase() {
    private lateinit var turnMotors: MotorGroup

    val velocity: Double
        get() = turnMotors.velocities[0] / GoBILDA.RPM_30.cpr * PI

    val angle: Double
        get() = turnMotors.positions[0] / GoBILDA.RPM_30.cpr * PI

    private var feedforward = ArmFeedforward(0.0, ArmConstants.kCos.value, 0.0);

    private val controller = ProfiledPIDController(
        ArmConstants.kP.value,
        ArmConstants.kI.value,
        ArmConstants.kD.value,
        TrapezoidProfile.Constraints(
            ArmConstants.MAX_VELOCITY.value,
            ArmConstants.MAX_ACCELERATION.value
        )
    )
    val isBusy
        get() = controller.atGoal()

    var setpoint = controller.goal.position
        set(value) {
            controller.goal = TrapezoidProfile.State(value, 0.0)
            field = value
        }

    private var enabled = true

    fun initialize(hardwareMap: HardwareMap) {
        val armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        val armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        armLeft.inverted = true

        turnMotors = MotorGroup(armLeft, armRight)

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
        turnMotors.set(0.0)
    }

    fun operateArm() {
        val output = controller.calculate(angle) + ArmConstants.kCos.value * cos(angle)

        if (enabled)
            turnMotors.set(output)
    }

//    @JvmField var kP = 0.0
//    @JvmField var kI = 0.0
//    @JvmField var kD = 0.0
//    @JvmField var kCos = 0.0
//    @JvmField var kS = 0.0
//    @JvmField var kV = 0.0
}