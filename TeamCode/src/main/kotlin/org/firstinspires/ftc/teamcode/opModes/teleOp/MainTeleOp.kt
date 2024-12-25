package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinUpCommand
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
    private var operatorMode = OPERATOR_MODE.MANUAL

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        ElevatorSubsystem.initialize(hardwareMap)
        ArmSubsystem.initialize(hardwareMap)
        DriveSubsystem.initialize(hardwareMap)
        IntakeSubsystem.initialize(hardwareMap)

//        spinUpCommand = ElevatorCommand(30.0, ElevatorSubsystem)
        spinUpCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to SpinUpCommand(ElevatorSubsystem),
                OPERATOR_MODE.SPECIMEN to ElevatorCommand(12.0, ElevatorSubsystem),
                OPERATOR_MODE.SAMPLE to ElevatorCommand(30.0, ElevatorSubsystem),
            ),
            this::operatorMode
        )
//        spinDownCommand = ElevatorCommand(0.0, ElevatorSubsystem)
        spinDownCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to SpinDownCommand(ElevatorSubsystem),
                OPERATOR_MODE.SAMPLE to ElevatorCommand(0.0, ElevatorSubsystem),
                OPERATOR_MODE.SPECIMEN to ElevatorCommand(0.0, ElevatorSubsystem),
            ),
            this::operatorMode
        )

//        armUpCommand = ArmCommand(Math.toRadians(90.0), ArmSubsystem)
        armUpCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to OpenArmCommand(ArmSubsystem, true),
                OPERATOR_MODE.SAMPLE to ArmCommand(90.0, ArmSubsystem),
                OPERATOR_MODE.SPECIMEN to ArmCommand(90.0, ArmSubsystem),
            ),
            this::operatorMode
        )
//        armDownCommand = ArmCommand(Math.toRadians(0.0), ArmSubsystem)
        armDownCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to OpenArmCommand(ArmSubsystem, false),
                OPERATOR_MODE.SAMPLE to ArmCommand(0.0, ArmSubsystem),
                OPERATOR_MODE.SPECIMEN to ArmCommand(0.0, ArmSubsystem),
            ),
            this::operatorMode
        )

        intakeCommand = IntakeCommand(true, IntakeSubsystem)
        outtakeCommand = IntakeCommand(false, IntakeSubsystem)

        intakeBeltCommand = IntakeBeltCommand(Math.toRadians(-60.0), IntakeSubsystem)
        outtakeBeltCommand = IntakeBeltCommand(Math.toRadians(90.0), IntakeSubsystem)

        driveCommand = DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(armUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(armDownCommand)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(spinDownCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(spinUpCommand)

//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(
//            InstantCommand({
//                if(operatorMode == OPERATOR_MODE.MANUAL)
//                    schedule(spinUpCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//            InstantCommand({
//                if(operatorMode != OPERATOR_MODE.MANUAL)
//                    schedule(spinUpCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(
//            InstantCommand({
//                if(operatorMode == OPERATOR_MODE.MANUAL)
//                    schedule(spinDownCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//            InstantCommand({
//                if(operatorMode != OPERATOR_MODE.MANUAL)
//                    schedule(spinDownCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
//            InstantCommand({
//                if(operatorMode == OPERATOR_MODE.MANUAL)
//                    schedule(armUpCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//            InstantCommand({
//                if(operatorMode != OPERATOR_MODE.MANUAL)
//                    schedule(armUpCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
//            InstantCommand({
//                if(operatorMode == OPERATOR_MODE.MANUAL)
//                    schedule(armDownCommand)
//            })
//        )
//
//        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//            InstantCommand({
//                if(operatorMode != OPERATOR_MODE.MANUAL)
//                    schedule(armDownCommand)
//            })
//        )

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

        driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
            InstantCommand({
                DriveSubsystem.driveMultiplier = 0.9
                driverMode = driverMode.toggle(driverMode)
            }),
            InstantCommand({
                DriveSubsystem.driveMultiplier = 0.5
                driverMode = driverMode.toggle(driverMode)
            })
        )

        DriveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position", ArmSubsystem.angle)

            telemetry.addData("Slides Position", ElevatorSubsystem.position)
            telemetry.addData("Slides Velocity", ElevatorSubsystem.velocity)

            telemetry.addData("Intake Position", Math.toDegrees(IntakeSubsystem.position))

            telemetry.addData("Operator Mode", operatorMode)
            telemetry.addData("Driver Mode", driverMode)

            telemetry.update()
        }).perpetually().schedule()
    }

    internal enum class OPERATOR_MODE {
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

    internal enum class DRIVER_MODE {
        SPEED,
        SLOW;

        fun toggle(mode: DRIVER_MODE) : DRIVER_MODE =
            when (mode) {
                SPEED -> SLOW
                SLOW -> SPEED
            }
    }
}

