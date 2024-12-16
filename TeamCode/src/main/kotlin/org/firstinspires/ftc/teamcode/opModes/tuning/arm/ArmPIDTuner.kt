package org.firstinspires.ftc.teamcode.opModes.tuning.arm

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem

@TeleOp
@Config
class ArmPIDTuner : CommandOpMode() {
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        ArmSubsystem.initialize(hardwareMap)

        RunCommand({
            ArmSubsystem.setpoint = Math.toRadians(target)
            ArmSubsystem.operateArm()
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Arm Angle", Math.toDegrees(ArmSubsystem.angle))
            telemetry.addData("Setpoint", Math.toDegrees(ArmSubsystem.setpoint))
            telemetry.update()
        }).perpetually().schedule()

        register(ArmSubsystem)
    }

    companion object {
        @JvmField
        // NOTE THAT THIS VALUE SHOULD BE DEGREES
        var target = 0.0
    }
}