package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.arm.ArmCommand
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinUpCommand
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

    private lateinit var spinUpCommand: Command
    private lateinit var spinDownCommand: Command

    private lateinit var armUpCommand: Command
    private lateinit var armDownCommand: Command

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

//        spinUpCommand = ElevatorCommand(30.0, ElevatorSubsystem)
        spinUpCommand = SpinUpCommand(ElevatorSubsystem)
//        spinDownCommand = ElevatorCommand(0.0, ElevatorSubsystem)
        spinDownCommand = SpinDownCommand(ElevatorSubsystem)

//        armUpCommand = ArmCommand(Math.toRadians(87.5), ArmSubsystem)
        armUpCommand = OpenArmCommand(ArmSubsystem, true)
//        armDownCommand = ArmCommand(Math.toRadians(0.0), ArmSubsystem)
        armDownCommand = OpenArmCommand(ArmSubsystem, false)

        intakeCommand = IntakeCommand(true, IntakeSubsystem)
        outtakeCommand = IntakeCommand(false, IntakeSubsystem)

        intakeBeltCommand = IntakeBeltCommand(true, intakeBeltSubsystem)
        outtakeBeltCommand = IntakeBeltCommand(false, intakeBeltSubsystem)

        driveCommand = DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(
            spinUpCommand
        )

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(
            spinDownCommand
        )

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(armUpCommand)

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(armDownCommand)

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            ConditionalCommand(
                intakeCommand,
                outtakeCommand,
                IntakeSubsystem::intakePos
            )
        )

        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            ConditionalCommand(
                intakeBeltCommand,
                outtakeBeltCommand,
                intakeBeltSubsystem::beltPos
            )
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