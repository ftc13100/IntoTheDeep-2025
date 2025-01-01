package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.TrapezoidProfileCommand
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.constants.ArmConstants
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

/**
 * Arm Command to set PID position for the arm
 * @see ArmSubsystem
 */
class ArmCommand(
    setpoint: Double,
    private val subsystem: ArmSubsystem
) : TrapezoidProfileCommand(
    TrapezoidProfile(
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
    ),
    subsystem::operateArm,
    subsystem
)