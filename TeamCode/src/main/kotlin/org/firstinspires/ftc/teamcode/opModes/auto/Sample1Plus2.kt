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

@Autonomous(name = "Sample 1 + 2")
class Sample1Plus2 : CommandOpMode() {
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
                            .splineToLinearHeading(Pose2d(55.0, 55.0, Math.toRadians(225.0)), Math.toRadians(45.0))                            .build(),
                        DriveSubsystem
                    ),
                    ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(2500),
                    InstantCommand({ IntakeSubsystem.closeClaw() }),
                ),
                ElevatorCommand(31.5, ElevatorSubsystem).withTimeout(2000),
                ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.0, 55.0, Math.toRadians(225.0))}
                            .strafeTo(Vector2d(59.0, 59.0))
                            .build()
                    ),
                ThrowItBackCommand(IntakeSubsystem),
                InstantCommand({ IntakeSubsystem.openClaw() }),
                WaitCommand(400),
                ActionCommand(
                    DriveSubsystem.actionBuilder{Pose2d(59.0, 59.0, Math.toRadians(225.0))}
                        .strafeTo(Vector2d(55.0, 55.0))
                        .build()
                ),
                )


        val scoreSample1 =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ),
                ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.0, 55.0, Math.toRadians(225.0))}
                            .turn(Math.toRadians(35.0))
                            .strafeTo(Vector2d(53.0, 52.0))
                            .build()
                    ),
                    ThrowItBackCommand(IntakeSubsystem),
                    ElevatorCommand(24.5, ElevatorSubsystem).withTimeout(1000),
                ),
                IntakeBeltCommand(Math.toRadians(-70.0), IntakeSubsystem).withTimeout(700),
                WaitCommand(250),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                WaitCommand(250),
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(53.0, 52.0, Math.toRadians(260.0))}
                            .turn(Math.toRadians(-35.0))
                            .strafeTo(Vector2d(55.0, 55.0))
                            .build()
                    ),
                ),
                ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(2500),
                ElevatorCommand(31.5, ElevatorSubsystem).withTimeout(2000),
                ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(55.0, 55.0, Math.toRadians(225.0))}
                            .strafeTo(Vector2d(59.0, 59.0))
                            .build()
                ),
                WaitCommand(250),
                ThrowItBackCommand(IntakeSubsystem),
                WaitCommand(250),
                InstantCommand({IntakeSubsystem.openClaw()}),
                ActionCommand(
                    DriveSubsystem.actionBuilder{Pose2d(59.0, 59.0, Math.toRadians(225.0))}
                        .strafeTo(Vector2d(55.0, 55.0))
                        .build()
                ),
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
                        DriveSubsystem.actionBuilder{Pose2d(55.0, 55.0, Math.toRadians(225.0))}
                            .turn(Math.toRadians(51.0))
                            .strafeTo(Vector2d(56.0, 53.0))
                            .build()
                    ),
                    WaitCommand(200),
                    ElevatorCommand(24.5, ElevatorSubsystem).withTimeout(1000),
                    ThrowItBackCommand(IntakeSubsystem),
                ),
                IntakeBeltCommand(Math.toRadians(-65.0), IntakeSubsystem).withTimeout(700),
                WaitCommand(100),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                WaitCommand(250),
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(56.0, 53.0, Math.toRadians(276.0))}
                            .turn(Math.toRadians(-51.0))
                            .strafeTo(Vector2d(54.0, 54.0))
                            .build()
                    ),
                ),
                WaitCommand(250),
                ArmCommand(Math.toRadians(93.0), ArmSubsystem).withTimeout(2000),
                ElevatorCommand(31.5, ElevatorSubsystem).withTimeout(2000),
                ThrowItBackCommand(IntakeSubsystem),
                WaitCommand(100),
                ActionCommand(
                    DriveSubsystem.actionBuilder{Pose2d(55.0, 55.0, Math.toRadians(225.0))}
                        .strafeTo(Vector2d(59.0, 59.0))
                        .build()
                ),
                WaitCommand(100),
                InstantCommand({IntakeSubsystem.openClaw()}),
                WaitCommand(500),
                ActionCommand(
                    DriveSubsystem.actionBuilder{Pose2d(59.0, 59.0, Math.toRadians(225.0))}
                        .strafeTo(Vector2d(53.0, 53.0))
                        .build()
                ),
                        ParallelCommandGroup(
                        IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(500),
                ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
            ),
        ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
            )





        /*val scoreSample3 =
            SequentialCommandGroup(
                ParallelCommandGroup(
                    IntakeBeltCommand(0.0, IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ),
                ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                ParallelCommandGroup(
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(56.0, 56.0, Math.toRadians(225.0))}
                            .turn(Math.toRadians(70.0))
                            .build()
                    ),
                    ElevatorCommand(24.0, ElevatorSubsystem).withTimeout(1000),
                    ThrowItBackCommand(IntakeSubsystem),
                ),
                IntakeBeltCommand(Math.toRadians(-50.0), IntakeSubsystem).withTimeout(700),
                WaitCommand(100),
                InstantCommand({IntakeSubsystem.closeClaw()}),
                WaitCommand(250),
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem).withTimeout(700),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                    ActionCommand(
                        DriveSubsystem.actionBuilder{Pose2d(56.0, 56.0, Math.toRadians(295.0))}
                            .turn(Math.toRadians(-70.0))
                            .build()
                    ),
                ),
                ArmCommand(Math.toRadians(91.0), ArmSubsystem).withTimeout(2000),
                ElevatorCommand(31.0, ElevatorSubsystem).withTimeout(1000),
                WaitCommand(100),
                InstantCommand({IntakeSubsystem.openClaw()}),
                WaitCommand(500),
                ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ArmCommand(0.0, ArmSubsystem)
                )
*/
        schedule(
            SequentialCommandGroup(
                scorePreloaded,
                scoreSample1,
                scoreSample2,
//                scoreSample3
            )
        )
    }

}