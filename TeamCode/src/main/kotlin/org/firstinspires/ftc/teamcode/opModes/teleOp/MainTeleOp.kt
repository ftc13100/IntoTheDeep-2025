package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinUpCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.OpenArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.OpenSlidesSubsystem

@TeleOp
class MainTeleOp : CommandOpMode() {
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor
    //private lateinit var intake: CRServo

    private lateinit var slidesSubsystem: OpenSlidesSubsystem
    private lateinit var armSubsystem: OpenArmSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem

    private lateinit var spinUpCommand: SpinUpCommand
    private lateinit var spinDownCommand: SpinDownCommand
    private lateinit var armUpCommand: OpenArmCommand
    private lateinit var armDownCommand: OpenArmCommand
    private lateinit var driveCommand: DriveCommand
    private lateinit var intakeCommand: IntakeCommand
    private lateinit var outtakeCommand: IntakeCommand


    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName, Motor.GoBILDA.RPM_435)
        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName, Motor.GoBILDA.RPM_435)

        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_435)
        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_435)

        //intake = CRServo(hardwareMap, ControlBoard.INTAKE.deviceName)

        slidesSubsystem = OpenSlidesSubsystem(elevatorRight, elevatorLeft)
        armSubsystem = OpenArmSubsystem(armRight, armLeft)
        DriveSubsystem.initialize(hardwareMap)
        //intakeSubsystem = IntakeSubsystem(intake)

        spinUpCommand = SpinUpCommand(slidesSubsystem)
        spinDownCommand = SpinDownCommand(slidesSubsystem)
        armUpCommand = OpenArmCommand(armSubsystem, true)
        armDownCommand = OpenArmCommand(armSubsystem, false)
        //intakeCommand = IntakeCommand(true, intakeSubsystem)
        //outtakeCommand = IntakeCommand(false, intakeSubsystem)
        driveCommand = DriveCommand(DriveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(armUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(armDownCommand)
        //operator.getGamepadButton(GamepadKeys.Button.Y).whenHeld(intakeCommand)
       // operator.getGamepadButton(GamepadKeys.Button.A).whileHeld(outtakeCommand)


        DriveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position: ", armSubsystem.armAngle)
            telemetry.update()
        }).perpetually().schedule()
    }
}