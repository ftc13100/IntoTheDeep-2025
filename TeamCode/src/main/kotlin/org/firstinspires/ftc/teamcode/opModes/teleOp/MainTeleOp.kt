package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeBeltCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeBeltSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@TeleOp
class MainTeleOp : CommandOpMode() {
    private lateinit var intakeBeltSubsystem: IntakeBeltSubsystem

    private lateinit var spinUpCommand: ElevatorCommand
    private lateinit var spinDownCommand: ElevatorCommand

    private lateinit var armUpCommand: ArmCommand
    private lateinit var armDownCommand: ArmCommand

    private lateinit var driveCommand: DriveCommand
    private lateinit var intakeCommand: IntakeCommand
    private lateinit var outtakeCommand: IntakeCommand
    private lateinit var intakeBeltCommand: IntakeBeltCommand
    private lateinit var outtakeBeltCommand: IntakeBeltCommand

    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        ElevatorSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        DriveSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

        intakeBeltSubsystem = IntakeBeltSubsystem(
            hardwareMap[Servo::class.java, ControlBoard.INTAKE_BELT.deviceName]
        )

        spinUpCommand = ElevatorCommand(30.0, ElevatorSubsystem)
        spinDownCommand = ElevatorCommand(0.0, ElevatorSubsystem)

        armUpCommand = ArmCommand(Math.toRadians(87.5), ArmSubsystem)
        armDownCommand = ArmCommand(Math.toRadians(0.0), ArmSubsystem)

        intakeCommand = IntakeCommand(true, IntakeSubsystem)
        outtakeCommand = IntakeCommand(false, IntakeSubsystem)

        intakeBeltCommand = IntakeBeltCommand(true, intakeBeltSubsystem)
        outtakeBeltCommand = IntakeBeltCommand(false, intakeBeltSubsystem)

        driveCommand = DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            spinUpCommand.withTimeout(1500)
        )

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            spinDownCommand.withTimeout(1500)
        )

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(armUpCommand)

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(armDownCommand)

        operator.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(

                intakeCommand,
                outtakeCommand,

        )

        operator.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(

                intakeBeltCommand,
                outtakeBeltCommand,

        )

        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
            InstantCommand(
                {
                    driveCommand = DriveCommand(
                        DriveSubsystem,
                        driver::getLeftX,
                        driver::getLeftY,
                        driver::getRightX,
                        0.0,
                        0.5
                    )
                }
            ),
            InstantCommand(
                {
                    driveCommand = DriveCommand(
                        DriveSubsystem,
                        driver::getLeftX,
                        driver::getLeftY,
                        driver::getRightX,
                        0.0,
                        0.9
                    )
                }
            ),
        )

        DriveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position", ArmSubsystem.angle)

            telemetry.addData("Slides Position", ElevatorSubsystem.position)
            telemetry.addData("Slides Velocity", ElevatorSubsystem.velocity)

            telemetry.update()
        }).perpetually().schedule()
    }
}