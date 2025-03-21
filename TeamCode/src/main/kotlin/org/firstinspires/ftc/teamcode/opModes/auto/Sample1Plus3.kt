package org.firstinspires.ftc.teamcode.opModes.auto

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
import org.firstinspires.ftc.teamcode.commands.intake.ThrowItBackCommand
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous(name = "Sample 1 + 3", preselectTeleOp = "MainTeleOp")
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
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(100),
                    ActionCommand(
                        DriveSubsystem.actionBuilder(AutoStartPose.BLUE_LEFT::startPose)
                            .setTangent(-45.0)
                            .splineToLinearHeading(Pose2d(55.5, 55.5, Math.toRadians(225.0)), Math.toRadians(45.0))                            .build(),
                        DriveSubsystem
                    ),
                    ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(4000),
                    InstantCommand({ IntakeSubsystem.closeClaw() }),
                ),
                ElevatorCommand(31.0, ElevatorSubsystem).withTimeout(2000),
                InstantCommand({ IntakeSubsystem.openClaw() }),
                WaitCommand(500),
            )


        val scoreSample1 =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ),
                ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.5, 55.5, Math.toRadians(225.0))}
                            .strafeToLinearHeading(Vector2d(53.5, 53.0), Math.toRadians(261.0))
                            .build()
                    ),
                    ElevatorCommand(24.0, ElevatorSubsystem).withTimeout(1000),
                ),
                IntakeBeltCommand(Math.toRadians(-65.0), IntakeSubsystem).withTimeout(700),
                WaitCommand(400),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                WaitCommand(400),
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(53.5, 53.0, Math.toRadians(261.0))}
                            .strafeToLinearHeading(Vector2d(55.5, 55.5), Math.toRadians(225.0))
                            .build()
                    ),
                ),
                ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(4000),
                ElevatorCommand(31.0, ElevatorSubsystem).withTimeout(1000),
                InstantCommand({IntakeSubsystem.openClaw()}),
                WaitCommand(500),
                )

        val scoreSample2 =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ),
                ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.5, 55.5, Math.toRadians(225.0))}
                            .strafeToLinearHeading(Vector2d(56.0, 53.0), Math.toRadians(283.0))
                            .build()
                    ),
                    ElevatorCommand(21.0, ElevatorSubsystem).withTimeout(1000),
                    ThrowItBackCommand(IntakeSubsystem),
                ),
                IntakeBeltCommand(Math.toRadians(-65.0), IntakeSubsystem).withTimeout(700),
                WaitCommand(100),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                WaitCommand(400),
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(56.0, 53.0, Math.toRadians(283.0))}
                            .strafeToLinearHeading(Vector2d(55.5, 55.5), Math.toRadians(225.0))
                            .build()
                    ),
                ),
                ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(2000),
                ElevatorCommand(31.0, ElevatorSubsystem).withTimeout(1000),
                InstantCommand({IntakeSubsystem.openClaw()}),
                WaitCommand(500),
            )

        val scoreSample3 =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ),
                ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.5, 55.5, Math.toRadians(225.0))}
                            .strafeToLinearHeading(Vector2d(56.5, 51.0), Math.toRadians(297.0))
                            .build()
                    ),
                    ElevatorCommand(23.0, ElevatorSubsystem).withTimeout(1000),
                    ThrowItBackCommand(IntakeSubsystem),
                ),
                IntakeBeltCommand(Math.toRadians(-65.0), IntakeSubsystem).withTimeout(700),
                WaitCommand(100),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                WaitCommand(400),
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(56.5, 51.0, Math.toRadians(297.0))}
                            .strafeToLinearHeading(Vector2d(55.5, 55.5), Math.toRadians(225.0))
                            .build()
                    ),
                ),
                ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(2000),
                ElevatorCommand(31.0, ElevatorSubsystem).withTimeout(1000),
                InstantCommand({IntakeSubsystem.openClaw()}),
                WaitCommand(500),
                ParallelCommandGroup(
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(400),
                ),
                ParallelCommandGroup(
                    IntakeBeltCommand(65.0, IntakeSubsystem).withTimeout(400),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.5, 55.5, Math.toRadians(225.0))}
                            .setTangent(Math.toRadians(-90.0))
                            .splineToLinearHeading(Pose2d(25.0, 10.0, 0.0), Math.toRadians(120.0))
                            .build()
                    )
                )
            )

        schedule(
            SequentialCommandGroup(
                scorePreloaded,
                scoreSample1,
                scoreSample2,
                scoreSample3
            )
        )
    }

}