package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.State
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

class DefaultArmCommand(
    private val subsystem: ArmSubsystem
) : CommandBase() {
    private val timer = ElapsedTime()
    private lateinit var profile: TrapezoidProfile
    private lateinit var prevState: State

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        timer.reset()

        prevState = State(
            subsystem.angle,
            subsystem.velocity
        )


        profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY.value,
                ArmConstants.MAX_ACCELERATION.value
            ),
            State(
                ArmSubsystem.target,
                0.0
            ),
            prevState
        )
    }

    override fun execute() {
        val state = profile.calculate(timer.seconds())
        timer.reset()

        subsystem.operateArm(state)

        prevState = state
        profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY.value,
                ArmConstants.MAX_ACCELERATION.value
            ),
            State(
                ArmSubsystem.target,
                0.0
            ),
            prevState
        )
    }
}