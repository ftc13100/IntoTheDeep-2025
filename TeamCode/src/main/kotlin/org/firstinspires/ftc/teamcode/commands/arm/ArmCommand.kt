package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

/**
 * Arm Command to set PID position for the arm
 * @see ArmSubsystem
 */
class ArmCommand(
    private val setpoint: Double,
    private val subsystem: ArmSubsystem
) : CommandBase() {
    override fun initialize() {
        subsystem.setpoint = setpoint
        addRequirements(subsystem)
    }

    override fun execute() = subsystem.operateArm()

    override fun isFinished(): Boolean = subsystem.isBusy

    override fun end(interrupted: Boolean) = subsystem.stop()
}