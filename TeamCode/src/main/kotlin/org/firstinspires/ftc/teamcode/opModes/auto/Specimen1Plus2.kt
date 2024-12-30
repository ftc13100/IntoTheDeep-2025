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

@Autonomous(name = "Specimen 1 + 2")
class Specimen1Plus2 : CommandOpMode() {
    override fun initialize() {
        DriveSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        ElevatorSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

        val scorePreloaded =
            SequentialCommandGroup(
                InstantCommand({ 
                    DriveSubsystem.pose = AutoStartPose.BLUE_RIGHT.startPose }),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder(AutoStartPose.BLUE_RIGHT::startPose)
                            .strafeToConstantHeading(Vector2d(2.0, 33.5))
                            .build(),
                        DriveSubsystem
                    ),
                    ArmCommand(Math.toRadians(91.0), ArmSubsystem).withTimeout(2000),
                    InstantCommand({ IntakeSubsystem.closeClaw() }),
                ),
                ElevatorCommand(11.0, ElevatorSubsystem).withTimeout(600),
                InstantCommand({ IntakeSubsystem.openClaw() }),
            )


        val pushSamples =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder { Pose2d(2.0, 33.5, Math.toRadians(90.0)) }
                            .setTangent(Math.toRadians(90.0))
                            .splineToLinearHeading(
                                Pose2d(-36.0, 35.0, Math.toRadians(90.0)),
                                Math.toRadians(-90.0)
                            )
                            .setTangent(Math.toRadians(-90.0))
                            //push one
                            .splineToLinearHeading(
                                Pose2d(-46.0, 15.0, Math.toRadians(90.0)),
                                Math.toRadians(120.0)
                            )
                            .strafeTo(Vector2d(-46.0, 56.0))
                            //push two
                            .setTangent(Math.toRadians(-90.0))
                            .splineToLinearHeading(
                                Pose2d(-58.0, 15.0, Math.toRadians(90.0)),
                                Math.toRadians(130.0)
                            )
                            .strafeTo(Vector2d(-58.0, 56.0))
                            //pick up
                            .setTangent(Math.toRadians(-90.0))
                            .splineToLinearHeading(
                                Pose2d(-34.0, 48.0, Math.toRadians(180.0)),
                                Math.toRadians(90.0)
                            )
                            .setTangent(Math.toRadians(90.0))
                            .strafeTo(Vector2d(-34.0, 59.0))
                            .build(),
                        DriveSubsystem
                    ),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(800),
                    ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                )
            )

        val scoreSpecimens =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(Math.toRadians(-60.0), IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(8.0, ElevatorSubsystem).withTimeout(500),
                ),
                WaitCommand(400),
                InstantCommand({ IntakeSubsystem.closeClaw() }),
                WaitCommand(500),
                ParallelCommandGroup(
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(500),
                    ArmCommand(Math.toRadians(91.0), ArmSubsystem).withTimeout(2000),
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(500),
                    ActionCommand(
                        DriveSubsystem.actionBuilder { Pose2d(-34.0, 59.0, Math.toRadians(180.0)) }
                            .setTangent(Math.toRadians(180.0))
                            .splineToLinearHeading(Pose2d(-2.0, 33.5, Math.toRadians(90.0)), Math.toRadians(-90.0))
                            .build(),
                        DriveSubsystem,
                    )
                ),
                ElevatorCommand(11.0, ElevatorSubsystem).withTimeout(700),
                InstantCommand({ IntakeSubsystem.openClaw() }),
                ParallelCommandGroup(
                    ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder { Pose2d(-2.0, 33.5, Math.toRadians(90.0)) }
                            .splineToLinearHeading(Pose2d(-34.0, 59.0, Math.toRadians(180.0)), 0.0)
                            .build()
                    ),
                )
            )
        val scoreSpecimens1 =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(Math.toRadians(-60.0), IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(8.0, ElevatorSubsystem).withTimeout(500),
                ),
                WaitCommand(400),
                InstantCommand({ IntakeSubsystem.closeClaw() }),
                WaitCommand(500),
                ParallelCommandGroup(
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(500),
                    ArmCommand(Math.toRadians(91.0), ArmSubsystem).withTimeout(2000),
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(500),
                    ActionCommand(
                        DriveSubsystem.actionBuilder { Pose2d(-34.0, 59.0, Math.toRadians(180.0)) }
                            .setTangent(Math.toRadians(180.0))
                            .splineToLinearHeading(Pose2d(-4.0, 33.5, Math.toRadians(90.0)), Math.toRadians(-90.0))
                            .build(),
                        DriveSubsystem,
                    )
                ),
                ElevatorCommand(11.0, ElevatorSubsystem).withTimeout(700),
                InstantCommand({ IntakeSubsystem.openClaw() }),
                ParallelCommandGroup(
                    ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder { Pose2d(-4.0, 33.5, Math.toRadians(90.0)) }
                            .strafeTo(Vector2d(-38.0, 59.0))
                            .build()
                    ),
                )
            )



        schedule(
            SequentialCommandGroup(
                scorePreloaded,
                pushSamples,
                scoreSpecimens,
                scoreSpecimens1,
            )
        )
    }

}