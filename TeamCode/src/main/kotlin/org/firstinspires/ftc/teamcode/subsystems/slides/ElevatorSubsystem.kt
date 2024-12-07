package org.firstinspires.ftc.teamcode.subsystems.slides

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.arcrobotics.ftclib.util.MathUtils.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.constants.SlidesConstants
import kotlin.math.sin

@Config
class ElevatorSubsystem(
    private val elevatorRight : Motor,
    elevatorLeft : Motor,
    private val slideAngle: () -> Double
) : SubsystemBase() {
    private val extendMotors = MotorGroup(elevatorRight, elevatorLeft)
    private val controller = ProfiledPIDController(
        SlidesConstants.kP.value,
        SlidesConstants.kI.value,
        SlidesConstants.kD.value,
        TrapezoidProfile.Constraints(
            30.0, // in/s
            30.0 // in/s^2
        )
    )

    var setpoint = 0.0
        set(value) {
            val clamped = if (slideAngle.invoke() < Math.toRadians(75.0))
                clamp(value, 0.0, 20.0)
            else
                clamp(value, 0.0, 30.0)

            controller.goal = TrapezoidProfile.State(clamped, 0.0)
            field = clamped
        }

    val slidePos: Double
        get() = extendMotors.positions[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    val slideVelocity: Double
        get() = -extendMotors.velocities[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value

    var enabled = true

    init {
        elevatorRight.inverted = true
        elevatorRight.encoder.setDirection(Motor.Direction.REVERSE)
        extendMotors.resetEncoder()
        extendMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun periodic() {
        controller.setPID(kP, kI, kD)

        val output = controller.calculate(slidePos)

        if (enabled)
            extendMotors.set(output + kG * sin(slideAngle.invoke()))
    }

    fun toggle() {
        enabled != enabled
    }

    fun spinUp() {
        extendMotors.set(1.0);
    }

    fun stop() {
        extendMotors.set(kG)
    }

    companion object {
        @JvmField
        var kP = 0.7
        // kP = 0.01

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var kG = 0.12
        // kG = 0.115

    }
}

