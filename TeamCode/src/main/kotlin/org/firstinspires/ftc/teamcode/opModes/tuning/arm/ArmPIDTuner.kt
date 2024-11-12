package org.firstinspires.ftc.teamcode.opModes.tuning.arm

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

class ArmPIDTuner : CommandOpMode() {
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor

    private lateinit var armSubsystem: ArmSubsystem

    override fun initialize() {
        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_435)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_435)

        armSubsystem = ArmSubsystem(armLeft, armRight)


    }
}