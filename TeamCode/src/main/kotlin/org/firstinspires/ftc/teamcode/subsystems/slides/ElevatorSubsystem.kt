package org.firstinspires.ftc.teamcode.subsystems.slides

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.arcrobotics.ftclib.util.MathUtils.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.SlidesConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import kotlin.math.sin

@Config
object ElevatorSubsystem: SubsystemBase() {
    private lateinit var elevatorMotors: MotorGroup

    private val controller = ProfiledPIDController(
        SlidesConstants.kP.value,
        SlidesConstants.kI.value,
        SlidesConstants.kD.value,
        TrapezoidProfile.Constraints(
            SlidesConstants.MAX_VELOCITY.value,
            SlidesConstants.MAX_ACCELERATION.value
        )
    )

    var setpoint = controller.goal.position
        set(value) {
            val clamped = if (ArmSubsystem.angle < Math.toRadians(75.0))
                clamp(value, 0.0, 20.0)
            else
                clamp(value, 0.0, 30.0)

            controller.goal = TrapezoidProfile.State(clamped, 0.0)
            field = clamped
        }

    val isBusy
        get() = controller.atGoal()

    val position: Double
        get() = elevatorMotors.positions[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    val velocity: Double
        get() = -elevatorMotors.velocities[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    var enabled = true

    fun initialize(hardwareMap: HardwareMap) {
        val elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        val elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        elevatorRight.inverted = true
        elevatorRight.encoder.setDirection(Motor.Direction.REVERSE)

        elevatorMotors = MotorGroup(elevatorRight, elevatorLeft)
        elevatorMotors.resetEncoder()
    }

    fun spinUp() {
        elevatorMotors.set(1.0)
    }

    fun spinDown() {
        elevatorMotors.set(-1.0)
    }

    fun stop() {
        elevatorMotors.set(SlidesConstants.kG.value * sin(ArmSubsystem.angle))
    }

    fun operateElevator() {
        val output = controller.calculate(position) +
                SlidesConstants.kG.value * sin(ArmSubsystem.angle)

        if (enabled) {
            elevatorMotors.set(output + SlidesConstants.kG.value * sin(ArmSubsystem.angle))
        }
    }

    fun toggle() {
        enabled != enabled
    }

    fun disable() {
        enabled = false
    }
}