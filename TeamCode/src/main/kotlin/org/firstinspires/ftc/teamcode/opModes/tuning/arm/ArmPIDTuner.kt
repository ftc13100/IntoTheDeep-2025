package org.firstinspires.ftc.teamcode.opModes.tuning.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.Constraints
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.State
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

@TeleOp
@Config
class ArmPIDTuner : OpMode() {
    private val timer = ElapsedTime()

    private lateinit var profile: TrapezoidProfile

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        ArmSubsystem.initialize(hardwareMap)

        profile = TrapezoidProfile(
            Constraints(
                ArmConstants.MAX_VELOCITY.value,
                ArmConstants.MAX_ACCELERATION.value
            ),
            State(
                Math.toRadians(target), 0.0
            ),
            State(
                ArmSubsystem.angle,
                ArmSubsystem.velocity
            )
        )
    }

    override fun start() {
        timer.reset()
    }

    override fun loop() {
        val time = timer.seconds()
        val state = profile.calculate(time)

        timer.reset()
        ArmSubsystem.operateArm(state)

        telemetry.addData("Target Position", Math.toDegrees(state.position))
        telemetry.addData("Target Velocity", Math.toDegrees(state.velocity))
        telemetry.addData("Measured Position", Math.toDegrees(ArmSubsystem.angle))
        telemetry.addData("Measured Velocity", Math.toDegrees(ArmSubsystem.velocity))

        telemetry.update()

        profile = TrapezoidProfile(
            Constraints(
                ArmConstants.MAX_VELOCITY.value,
                ArmConstants.MAX_ACCELERATION.value
            ),
            State(
                Math.toRadians(target), 0.0
            ),
            state
        )
    }

    companion object {
        @JvmField
        // NOTE THAT THIS VALUE SHOULD BE DEGREES
        var target = 0.0
    }
}