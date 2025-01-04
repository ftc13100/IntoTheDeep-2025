package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

/**
 * Arm Command to set PID position for the arm
 * @see ArmSubsystem
 */
class ArmCommand(
    private val setpoint: Double,
    private val subsystem: ArmSubsystem
) : CommandBase() {
    private val timer = ElapsedTime()
    private lateinit var profile: TrapezoidProfile

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        timer.reset()

        profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY.value,
                ArmConstants.MAX_ACCELERATION.value
            ),
            TrapezoidProfile.State(
                setpoint,
                0.0
            ),
            TrapezoidProfile.State(
                subsystem.angle,
                subsystem.velocity
            )
        )
    }

    override fun execute() {
        val state = profile.calculate(timer.seconds())

        subsystem.operateArm(state)
    }

    override fun end(interrupted: Boolean) {
        subsystem.stop()
    }

    override fun isFinished(): Boolean {
        return timer.seconds() >= profile.totalTime()
    }
}