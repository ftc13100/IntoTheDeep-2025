package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SelectCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.kotlin.extensions.gamepad.and
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinUpCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeBeltCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.commands.intake.SlowIntakeBeltCommand
import org.firstinspires.ftc.teamcode.commands.intake.ThrowItBackCommand
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
    private lateinit var intakeCommand: Command
    private lateinit var outtakeCommand: Command
    private lateinit var intakeBeltCommand: Command
    private lateinit var outtakeBeltCommand: Command
    private lateinit var slowIntakeBeltCommand: Command
    private lateinit var slowOuttakeBeltCommand: Command

    private lateinit var operatorLeft: Trigger
    private lateinit var operatorRight: Trigger



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
                OPERATOR_MODE.SPECIMEN to ConditionalCommand(
                    ElevatorCommand(12.0, ElevatorSubsystem).withTimeout(2500),
                    ElevatorCommand(20.0, ElevatorSubsystem).withTimeout(2500)
                ) { ArmSubsystem.angle > Math.toRadians(80.0) },
                OPERATOR_MODE.SAMPLE to ConditionalCommand(
                    ElevatorCommand(30.0, ElevatorSubsystem).withTimeout(2500),
                    ElevatorCommand(20.0, ElevatorSubsystem).withTimeout(2500)
                ) { ArmSubsystem.angle > Math.toRadians(80.0) },
            ),
            this::operatorMode
        )
//        spinDownCommand = ElevatorCommand(0.0, ElevatorSubsystem)
        spinDownCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to SpinDownCommand(ElevatorSubsystem),
                OPERATOR_MODE.SAMPLE to ElevatorCommand(Math.toRadians(0.0), ElevatorSubsystem).withTimeout(2500),
                OPERATOR_MODE.SPECIMEN to ElevatorCommand(Math.toRadians(0.0), ElevatorSubsystem).withTimeout(2500),
            ),
            this::operatorMode
        )

//        armUpCommand = ArmCommand(Math.toRadians(90.0), ArmSubsystem)
        armUpCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to OpenArmCommand(ArmSubsystem, true),
                OPERATOR_MODE.SAMPLE to ArmCommand(Math.toRadians(90.0), ArmSubsystem).withTimeout(2500),
                OPERATOR_MODE.SPECIMEN to ArmCommand(Math.toRadians(90.0), ArmSubsystem).withTimeout(2500),
            ),
            this::operatorMode
        )
//        armDownCommand = ArmCommand(Math.toRadians(0.0), ArmSubsystem)
        armDownCommand = SelectCommand(
            mapOf(
                OPERATOR_MODE.MANUAL to OpenArmCommand(ArmSubsystem, false),
                OPERATOR_MODE.SAMPLE to ArmCommand(0.0, ArmSubsystem).withTimeout(2500),
                OPERATOR_MODE.SPECIMEN to ArmCommand(0.0, ArmSubsystem).withTimeout(2500),
            ),
            this::operatorMode
        )

        intakeCommand = IntakeCommand(true, IntakeSubsystem)
        outtakeCommand = IntakeCommand(false, IntakeSubsystem)

        intakeBeltCommand = IntakeBeltCommand(Math.toRadians(-45.0), IntakeSubsystem)
//        outtakeBeltCommand = IntakeBeltCommand(Math.toRadians(90.0), IntakeSubsystem)
        outtakeBeltCommand = ThrowItBackCommand(IntakeSubsystem)
        slowIntakeBeltCommand = SlowIntakeBeltCommand(IntakeSubsystem, true)
        slowOuttakeBeltCommand = SlowIntakeBeltCommand(IntakeSubsystem, false)

        driveCommand = DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)


        (operator.getGamepadButton(GamepadKeys.Button.DPAD_UP) and Trigger {
            operatorMode == OPERATOR_MODE.MANUAL
        }) .whileActiveOnce(spinUpCommand)

        (operator.getGamepadButton(GamepadKeys.Button.DPAD_UP) and Trigger {
            operatorMode != OPERATOR_MODE.MANUAL
        }) .whenActive(spinUpCommand)

        (operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN) and Trigger {
            operatorMode == OPERATOR_MODE.MANUAL
        }).whileActiveOnce(spinDownCommand)

        (operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN) and Trigger {
            operatorMode != OPERATOR_MODE.MANUAL
        }) .whenActive(spinDownCommand)


        (operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER) and Trigger {
            operatorMode == OPERATOR_MODE.MANUAL
        }) .whileActiveOnce(armUpCommand)

        (operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER) and Trigger {
            operatorMode != OPERATOR_MODE.MANUAL
        }) .whenActive(armUpCommand)

        (operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) and Trigger {
            operatorMode == OPERATOR_MODE.MANUAL
        }) .whileActiveOnce(armDownCommand)

        (operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER) and Trigger {
            operatorMode != OPERATOR_MODE.MANUAL
        }) .whenActive(armDownCommand)


        operator.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
            intakeCommand,
            outtakeCommand,
        )

        operator.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
            intakeBeltCommand,
            outtakeBeltCommand,
        )

        operatorLeft = Trigger {
            operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0
        }


        operatorRight = Trigger {
            operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0
        }

        operatorRight.whileActiveContinuous(
            slowOuttakeBeltCommand
        )

        operatorLeft.whileActiveContinuous(
            slowIntakeBeltCommand
        )

        (operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT) and Trigger {
            operatorMode != OPERATOR_MODE.MANUAL
        }) .whenActive(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    ThrowItBackCommand(IntakeSubsystem),
                    ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000)
                ),
                ArmCommand(Math.toRadians(89.0), ArmSubsystem).withTimeout(2000)
            )
        )

        (operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT) and Trigger {
            operatorMode != OPERATOR_MODE.MANUAL
        }) .whenActive(
            SequentialCommandGroup(
                ElevatorCommand(0.0, ElevatorSubsystem).withTimeout(1000),
                ArmCommand(0.0, ArmSubsystem).withTimeout(2000),
                ParallelCommandGroup(
                    IntakeBeltCommand(-45.0, IntakeSubsystem).withTimeout(500),
                    ElevatorCommand(17.0, ElevatorSubsystem)
                ),

            )
        )


        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            InstantCommand({ operatorMode = operatorMode.toggle(operatorMode)})
        )

        driver.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
            InstantCommand({
                DriveSubsystem.driveMultiplier = 0.35
                driverMode = driverMode.toggle(driverMode)
            }),
            InstantCommand({
                DriveSubsystem.driveMultiplier = 0.9
                driverMode = driverMode.toggle(driverMode)
            })
        )

        DriveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position", ArmSubsystem.angle)

            telemetry.addData("Slides Position", ElevatorSubsystem.position)
            telemetry.addData("Slides Velocity", ElevatorSubsystem.velocity)

            telemetry.addData("Intake Position", Math.toDegrees(IntakeSubsystem.position))
            telemetry.addData("Intake at limit?", IntakeSubsystem.isPressed)

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

