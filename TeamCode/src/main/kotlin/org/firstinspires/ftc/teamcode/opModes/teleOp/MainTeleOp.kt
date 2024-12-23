package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeBeltCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@TeleOp
class MainTeleOp : CommandOpMode() {
    private lateinit var spinUpCommand: Command
    private lateinit var spinDownCommand: Command

    private lateinit var armUpCommand: Command
    private lateinit var armDownCommand: Command

    private lateinit var driveCommand: Command
    private lateinit var intakeCommand: IntakeCommand
    private lateinit var outtakeCommand: IntakeCommand
    private lateinit var intakeBeltCommand: IntakeBeltCommand
    private lateinit var outtakeBeltCommand: IntakeBeltCommand

    private lateinit var driver: GamepadEx
    private var driverMode = DRIVER_MODE.SPEED

    private lateinit var operator: GamepadEx
    private var operatorMode = OPERATOR_MODE.SAMPLE

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        ElevatorSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        DriveSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

        spinUpCommand = ElevatorCommand(30.0, ElevatorSubsystem)
        spinDownCommand = ElevatorCommand(0.0, ElevatorSubsystem)

        armUpCommand = ArmCommand(Math.toRadians(90.0), ArmSubsystem)
        armDownCommand = ArmCommand(Math.toRadians(0.0), ArmSubsystem)

        intakeCommand = IntakeCommand(true, IntakeSubsystem)
        outtakeCommand = IntakeCommand(false, IntakeSubsystem)

        intakeBeltCommand = IntakeBeltCommand(true, IntakeSubsystem)
        outtakeBeltCommand = IntakeBeltCommand(false, IntakeSubsystem)

        driveCommand = DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            spinUpCommand.withTimeout(1500)
        )

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            spinDownCommand.withTimeout(1500)
        )

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).run {
            when (operatorMode) {
                OPERATOR_MODE.MANUAL -> whenHeld(armUpCommand)
                else -> whenPressed(armUpCommand)
            }
        }

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).run {
            when (operatorMode) {
                OPERATOR_MODE.MANUAL -> whenHeld(armDownCommand)
                else -> whenPressed(armDownCommand)
            }
        }

        operator.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
            intakeCommand,
            outtakeCommand,
        )

        operator.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
            intakeBeltCommand,
            outtakeBeltCommand,
        )

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            InstantCommand({ operatorMode = operatorMode.toggle(operatorMode)})
        )

        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
            InstantCommand({
                driveCommand =
                    DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)
            }).andThen(
                InstantCommand({
                    driverMode = driverMode.toggle(driverMode)
                })
            ),

            InstantCommand({
                driveCommand =
                    DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)
            }).andThen(
                InstantCommand({
                    driverMode = driverMode.toggle(driverMode)
                })
            )
        )

        DriveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position", ArmSubsystem.angle)

            telemetry.addData("Slides Position", ElevatorSubsystem.position)
            telemetry.addData("Slides Velocity", ElevatorSubsystem.velocity)

            telemetry.addData("Operator Mode", operatorMode)
            telemetry.addData("Driver Mode", operatorMode)

            telemetry.update()
        }).perpetually().schedule()
    }

    enum class OPERATOR_MODE {
        MANUAL,
        SPECIMEN,
        SAMPLE;

        fun toggle(mode: OPERATOR_MODE) : OPERATOR_MODE =
            when (mode) {
                MANUAL -> SPECIMEN
                SPECIMEN -> SAMPLE
                SAMPLE -> MANUAL
            }
    }

    enum class DRIVER_MODE {
        SPEED,
        SLOW;

        fun toggle(mode: DRIVER_MODE) : DRIVER_MODE =
            when (mode) {
                SPEED -> SLOW
                SLOW -> SPEED
            }
    }
}

