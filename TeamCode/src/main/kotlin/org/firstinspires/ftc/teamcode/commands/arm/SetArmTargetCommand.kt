package org.firstinspires.ftc.teamcode.commands.arm

import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

class SetArmTargetCommand(
    target: Double
) : InstantCommand({
    ArmSubsystem.enabled = true
    ArmSubsystem.target = target
})