package org.firstinspires.ftc.teamcode.opModes.tuning.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

@TeleOp
@Config
class ArmPIDTuner : CommandOpMode() {
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor

    private lateinit var armSubsystem: ArmSubsystem

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_435)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_435)

        armSubsystem = ArmSubsystem(armLeft, armRight)

        InstantCommand({
            armSubsystem.setpoint = Math.toRadians(target)
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Arm Angle: ", armSubsystem.armAngle)
            telemetry.addData("Setpoint: ", target)
        })
    }

    companion object {
        @JvmField
        // NOTE THAT THIS VALUE SHOULD BE DEGREES
        var target = 0.0
    }
}