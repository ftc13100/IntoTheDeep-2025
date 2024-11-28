package org.firstinspires.ftc.teamcode.subsystems.slides

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.constants.SlidesConstants

@Config
class ElevatorSubsystem(
    elevatorRight : Motor,
    elevatorLeft : Motor
) : SubsystemBase() {
    private val extendMotors = MotorGroup(elevatorRight, elevatorLeft)
    private val controller = ProfiledPIDController(
        SlidesConstants.kP.value,
        SlidesConstants.kI.value,
        SlidesConstants.kD.value,
        TrapezoidProfile.Constraints(
            0.0,
            0.0
        )
    )

    var setpoint = 0.0
        set(value) {
            controller.goal = TrapezoidProfile.State(value, 0.0)
            field = value
        }

    val slidePos: Double
        get() = extendMotors.positions[0]

    val slideVelocity: Double
        get() = extendMotors.velocities[0]

    init {
        elevatorRight.inverted = true
        elevatorRight.encoder.setDirection(Motor.Direction.REVERSE)

        extendMotors.resetEncoder()
        extendMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    override fun periodic() {
        controller.setPID(kP, kI, kD)
        val output = controller.calculate(slidePos)

        extendMotors.set(output)
    }

    companion object {
        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0
    }
}