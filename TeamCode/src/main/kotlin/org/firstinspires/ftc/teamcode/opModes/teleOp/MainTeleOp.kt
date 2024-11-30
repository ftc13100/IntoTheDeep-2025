package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.arm.OpenArmCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinDownCommand
import org.firstinspires.ftc.teamcode.commands.elevator.SpinUpCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeBeltCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.OpenArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeBeltSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.OpenElevatorSubsystem

@TeleOp
class MainTeleOp : CommandOpMode() {
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor

    private lateinit var intake: Servo
    private lateinit var intakeBelt: Servo

    private lateinit var slidesSubsystem: OpenElevatorSubsystem
    private lateinit var armSubsystem: OpenArmSubsystem
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var intakeBeltSubsystem: IntakeBeltSubsystem


    private lateinit var spinUpCommand: SpinUpCommand
    private lateinit var spinDownCommand: SpinDownCommand

    private lateinit var armUpCommand: OpenArmCommand
    private lateinit var armDownCommand: OpenArmCommand

    private lateinit var driveCommand: DriveCommand

    private lateinit var intakeCommand: IntakeBeltCommand
    private lateinit var outtakeCommand: IntakeBeltCommand

    private lateinit var clawCommand: ConditionalCommand

    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx

    private lateinit var operatorLeft: Trigger
    private lateinit var operatorRight: Trigger

    override fun initialize() {
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName, Motor.GoBILDA.RPM_1150)
        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName, Motor.GoBILDA.RPM_1150)

        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName, Motor.GoBILDA.RPM_60)
        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName, Motor.GoBILDA.RPM_60)

        intake = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE.deviceName)
        intakeBelt = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE_BELT.deviceName)

        armSubsystem = OpenArmSubsystem(armRight, armLeft)
        slidesSubsystem = OpenElevatorSubsystem(elevatorRight, elevatorLeft, armSubsystem::armAngle)
        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intake)
        intakeBeltSubsystem = IntakeBeltSubsystem(intakeBelt)

        spinUpCommand = SpinUpCommand(slidesSubsystem)
        spinDownCommand = SpinDownCommand(slidesSubsystem)

        armUpCommand = OpenArmCommand(armSubsystem, true)
        armDownCommand = OpenArmCommand(armSubsystem, false)

        intakeCommand = IntakeBeltCommand(true, intakeBeltSubsystem)
        outtakeCommand = IntakeBeltCommand(false, intakeBeltSubsystem)

        clawCommand = ConditionalCommand(
            InstantCommand({ intakeSubsystem.intake() }),
            InstantCommand({ intakeSubsystem.outtake() }),
            intakeSubsystem::intakePos
        )

        driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, 0.0)

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(spinUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(spinDownCommand)
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(armUpCommand)
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(armDownCommand)

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(clawCommand)

        operatorLeft = Trigger {
            operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0.0
        }

        operatorRight = Trigger {
            operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0.0
        }

        operatorLeft.whileActiveContinuous(outtakeCommand)
        operatorRight.whileActiveContinuous(intakeCommand)

        driveSubsystem.defaultCommand = driveCommand

        RunCommand({
            telemetry.addData("Arm Position: ", Math.toDegrees(armSubsystem.armAngle))
            telemetry.addData("Intake Position", intakeSubsystem.intakePos)
            telemetry.update()
        }).perpetually().schedule()
    }
}