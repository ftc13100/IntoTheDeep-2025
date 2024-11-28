package org.firstinspires.ftc.teamcode.opModes.tuning.slides

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@TeleOp
@Config
class SlidesPIDTuner : CommandOpMode() {
    private lateinit var slidesLeft: Motor
    private lateinit var slidesRight: Motor

    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor

    private lateinit var armSubsystem: ArmSubsystem
    private lateinit var slidesSubsystem: ElevatorSubsystem

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_60)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_60)

        slidesLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName, Motor.GoBILDA.RPM_1150)
        slidesRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName, Motor.GoBILDA.RPM_1150)

        armSubsystem = ArmSubsystem(armRight, armLeft)
        slidesSubsystem = ElevatorSubsystem(slidesRight, slidesLeft, armSubsystem::armAngle)

        RunCommand({
            slidesSubsystem.setpoint = target
        }).perpetually().schedule()

        RunCommand({
            telemetry.addData("Slides Position", slidesSubsystem.slidePos)
            telemetry.addData("Setpoint", target)
            telemetry.update()
        }).perpetually().schedule()

        register(slidesSubsystem)
    }

    companion object {
        @JvmField
        // NOTE THAT THIS VALUE SHOULD BE DEGREES
        var target = 0.0
    }
}