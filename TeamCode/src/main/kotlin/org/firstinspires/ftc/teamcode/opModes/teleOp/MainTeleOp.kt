package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.ArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.drive.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.drive.SpinUpCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.SlidesSubsystem

@TeleOp
class MainTeleOp: CommandOpMode() {

    private lateinit var elevatorLeft : Motor
    private lateinit var elevatorRight : Motor
    private lateinit var armLeft : Motor
    private lateinit var armRight : Motor


    private lateinit var slidesSubsystem: SlidesSubsystem
    private lateinit var armSubsystem: ArmSubsystem
    private lateinit var driveSubsystem: DriveSubsystem

    private lateinit var spinUpCommand: SpinUpCommand
    private lateinit var spinDownCommand: SpinDownCommand
    private lateinit var armUpCommand: ArmCommand
    private lateinit var armDownCommand: ArmCommand
    private lateinit var driveCommand: DriveCommand

    private lateinit var driver : GamepadEx
    private lateinit var operator : GamepadEx

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)
        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_435)
        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_435)

        slidesSubsystem = SlidesSubsystem(elevatorRight, elevatorLeft)
        armSubsystem = ArmSubsystem(armRight, armLeft)
        driveSubsystem = DriveSubsystem(hardwareMap)

        spinUpCommand = SpinUpCommand(slidesSubsystem)
        spinDownCommand = SpinDownCommand(slidesSubsystem)
        armUpCommand = ArmCommand(armSubsystem, true)
        armDownCommand = ArmCommand(armSubsystem, false)
        driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(armUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(armDownCommand)

        driveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position: ", armSubsystem.armAngle)
            telemetry.update()
        }).perpetually().schedule()
    }
}