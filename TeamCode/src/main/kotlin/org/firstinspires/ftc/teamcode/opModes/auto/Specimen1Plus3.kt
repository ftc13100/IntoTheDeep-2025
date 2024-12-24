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

@Autonomous(name = "Specimen 1 + 3")
class Specimen1Plus3 : CommandOpMode() {
    override fun initialize() {
        DriveSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        ElevatorSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

        val scorePreloaded =
            SequentialCommandGroup(
                InstantCommand({ DriveSubsystem.pose = AutoStartPose.BLUE_RIGHT.startPose }),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder(AutoStartPose.BLUE_RIGHT::startPose)
                            .strafeToConstantHeading(Vector2d(2.0, 31.5))
                        .build(),
                        DriveSubsystem
                    ),
                    ArmCommand(Math.toRadians(90.0), ArmSubsystem).withTimeout(2000),
                    InstantCommand({ IntakeSubsystem.closeClaw() }),
                ),
                ElevatorCommand(12.0, ElevatorSubsystem).withTimeout(1250),
                InstantCommand({ IntakeSubsystem.openClaw() }),
            )

        val pushSamples =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder { Pose2d(2.0, 31.5, Math.toRadians(90.0)) }
                            .setTangent(Math.toRadians(90.0))
                            .splineToSplineHeading(Pose2d(-36.0, 35.0, 0.0), Math.toRadians(-90.0))
                            .setTangent(Math.toRadians(-90.0))
                            .splineToSplineHeading(Pose2d(-44.0, 5.0, 0.0), Math.toRadians(120.0))
                            .strafeTo(Vector2d(-44.0, 56.0))
                            .setTangent(Math.toRadians(-90.0))
                            .splineToSplineHeading( Pose2d(-56.0, 5.0, 0.0), Math.toRadians(130.0))
                            .strafeTo(Vector2d(-56.0, 56.0))
                            .setTangent(Math.toRadians(-90.0))
                            .splineToSplineHeading(Pose2d(-36.0, 59.0, Math.toRadians(180.0)), 0.0)
                        .build(),
                        DriveSubsystem
                    ),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1250),
                    ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                    IntakeBeltCommand(Math.toRadians(-60.0), IntakeSubsystem),
                )
            )

        val scoreSpecimens =
            SequentialCommandGroup(
                ElevatorCommand(5.0, ElevatorSubsystem).withTimeout(500),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                ParallelCommandGroup(
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(500),
                    ArmCommand(Math.toRadians(90.0), ArmSubsystem).withTimeout(2000),
                    IntakeBeltCommand(Math.toRadians(90.0), IntakeSubsystem)
                ),
                ActionCommand(
                    DriveSubsystem.actionBuilder { Pose2d(-36.0, 59.0, Math.toRadians(180.0)) }
                        .setTangent(Math.toRadians(180.0))
                        .splineToLinearHeading(Pose2d(-2.0, 31.5, Math.toRadians(90.0)), Math.toRadians(-90.0))
                        .build(),
                    DriveSubsystem,
                )
            )

        schedule(
            scorePreloaded.andThen(
                pushSamples.andThen(
                    scoreSpecimens
                )
            )
        )
    }

}