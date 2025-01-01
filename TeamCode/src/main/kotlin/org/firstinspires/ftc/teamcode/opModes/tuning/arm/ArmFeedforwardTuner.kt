package org.firstinspires.ftc.teamcode.opModes.tuning.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

@Config
@Autonomous
class ArmFeedforwardTuner: OpMode() {
    private lateinit var profile: TrapezoidProfile
    private var movingForward = true
    private var timer = ElapsedTime()

    companion object {
        @JvmField var angle = Math.toRadians(90.0)
    }

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        ArmSubsystem.initialize(hardwareMap, telemetry)

        profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY.value,
                ArmConstants.MAX_ACCELERATION.value
            ),
            TrapezoidProfile.State(
                angle, 0.0
            ),
            TrapezoidProfile.State(
                0.0, 0.0
            )
        )
    }

    override fun start() {
        timer.reset()
    }

    override fun loop() {
        val time = timer.seconds()

        if (time < profile.totalTime()) {
            val state = profile.calculate(time)
            ArmSubsystem.operateArm(state)
        } else {
            movingForward = !movingForward

            profile = TrapezoidProfile(
                TrapezoidProfile.Constraints(
                    ArmConstants.MAX_VELOCITY.value,
                    ArmConstants.MAX_ACCELERATION.value
                ),
                TrapezoidProfile.State(
                    if (movingForward) angle else 0.0, 0.0
                ),
                TrapezoidProfile.State(
                    ArmSubsystem.angle,
                    ArmSubsystem.velocity
                )
            )

            timer.reset()
        }
    }
}