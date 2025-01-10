package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.roadrunner.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.ActionCommand
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous(name = "Generic Auto")
class GenericAuto : CommandOpMode() {
    override fun initialize() {
        DriveSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        ElevatorSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

        val scorePreloaded =
            SequentialCommandGroup(
                ActionCommand(
                    DriveSubsystem.actionBuilder(AutoStartPose.BLUE_LEFT::startPose)
                        .splineToLinearHeading(
                            Pose2d(54.0, 56.0, Math.toRadians(225.0)),
                            Math.toRadians(0.0)
                        )
                        .build(),
                    DriveSubsystem
                ),
                ArmCommand(90.0, ArmSubsystem),
                ElevatorCommand(30.0, ElevatorSubsystem),
                IntakeCommand(true, IntakeSubsystem),
                WaitCommand(500),
                IntakeCommand(false, IntakeSubsystem),
                ElevatorCommand(0.0, ElevatorSubsystem),
                ArmCommand(0.0, ArmSubsystem),
            )

        val scoreFirst =
            SequentialCommandGroup(
                ActionCommand(
                    DriveSubsystem.actionBuilder(DriveSubsystem::pose)
                        .turn(Math.toRadians(37.0))
                        .build(),
                    DriveSubsystem
                ),
                ElevatorCommand(20.0, ElevatorSubsystem),
                IntakeCommand(true, IntakeSubsystem),
                WaitCommand(500),
                IntakeCommand(false, IntakeSubsystem),
                ElevatorCommand(0.0, ElevatorSubsystem),
                ArmCommand(90.0, ArmSubsystem),
                ElevatorCommand(30.0, ElevatorSubsystem)
            )

        schedule(
            scorePreloaded
            .andThen(
                scoreFirst
            )
        )
    }

}