package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.ActionCommand
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeBeltCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous(name = "Sample 1 + 3")
class Sample1Plus3 : CommandOpMode() {
    override fun initialize() {
        DriveSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        ElevatorSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

        val scorePreloaded =
            SequentialCommandGroup(
                InstantCommand({ DriveSubsystem.pose = AutoStartPose.BLUE_LEFT.startPose }),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder(AutoStartPose.BLUE_LEFT::startPose)
                            .setTangent(-45.0)
                            .splineToLinearHeading(Pose2d(56.0, 56.0, Math.toRadians(225.0)), Math.toRadians(45.0))                            .build(),
                        DriveSubsystem
                    ),
                    ArmCommand(Math.toRadians(90.0), ArmSubsystem).withTimeout(2000),
                    IntakeBeltCommand(Math.toRadians(-60.0),IntakeSubsystem),
                    InstantCommand({ IntakeSubsystem.closeClaw() }),
                ),
                ElevatorCommand(30.0, ElevatorSubsystem).withTimeout(2000),
                IntakeBeltCommand(Math.toRadians(90.0), IntakeSubsystem).withTimeout(1000),
                InstantCommand({ IntakeSubsystem.openClaw() }),
            )

        schedule(
            scorePreloaded
        )
    }

}