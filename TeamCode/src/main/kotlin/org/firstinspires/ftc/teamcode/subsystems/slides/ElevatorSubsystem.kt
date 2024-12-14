package org.firstinspires.ftc.teamcode.subsystems.slides

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.SlidesConstants

object ElevatorSubsystem: SubsystemBase() {
//    private lateinit var elevatorMotors: MotorGroup
    private lateinit var elevator: DcMotorEx

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
//            val clamped = if (ArmSubsystem.angle < Math.toRadians(75.0))
//                clamp(value, 0.0, 20.0)
//            else
//                clamp(value, 0.0, 30.0)
//
//            controller.goal = TrapezoidProfile.State(clamped, 0.0)
//            field = clamped
            elevator.targetPosition = value.toInt()
            field = value
        }

    val position: Double
//        get() = elevatorMotors.positions[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value
        get() = elevator.currentPosition.toDouble()

    val velocity: Double
//        get() = -elevatorMotors.velocities[0] * SlidesConstants.MAX_HEIGHT_INCH.value / SlidesConstants.MAX_HEIGHT_TICKS.value
        get() = elevator.velocity

    var enabled = true

    fun initialize(hardwareMap: HardwareMap) {
//        val elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
//        val elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        val elevator = hardwareMap[DcMotorEx::class.java, ControlBoard.SLIDES_RIGHT.deviceName]
        elevator.mode = DcMotor.RunMode.RUN_TO_POSITION

        elevator.power = 1.0
//        elevatorLeft.inverted = true
//        elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)
    }

    fun spinUp() {
//        elevatorMotors.set(1.0)
    }

    fun spinDown() {
//        elevatorMotors.set(-1.0)
    }

    fun stop() {
//        elevatorMotors.set(SlidesConstants.kG.value * sin(ArmSubsystem.angle))
    }

    override fun periodic() {
//        controller.setPID(kP, kI, kD)

//        val output = controller.calculate(position)

//        if (enabled)
//            elevatorMotors.set(output + SlidesConstants.kG.value * sin(ArmSubsystem.angle))
    }

    fun toggle() {
        enabled != enabled
    }

    fun disable() {
        enabled = false
    }
}