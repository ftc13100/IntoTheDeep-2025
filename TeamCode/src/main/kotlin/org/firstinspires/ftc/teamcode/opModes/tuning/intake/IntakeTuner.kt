package org.firstinspires.ftc.teamcode.opModes.tuning.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@TeleOp
@Config
class IntakeTuner : CommandOpMode() {
    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        IntakeSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)

        RunCommand({
            IntakeSubsystem.setClaw(claw)
        }).perpetually().schedule()

        RunCommand({
            IntakeSubsystem.target = Math.toRadians(wristTarget)

            IntakeSubsystem.operateWrist()
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Intake Position", Math.toDegrees(IntakeSubsystem.position))
            telemetry.addData("Intake Velocity", Math.toDegrees(IntakeSubsystem.velocity))

            telemetry.update()
        }).perpetually().schedule()
    }

    companion object {
        @JvmField var claw = 0.75
        @JvmField var wristTarget = 0.0
    }
}