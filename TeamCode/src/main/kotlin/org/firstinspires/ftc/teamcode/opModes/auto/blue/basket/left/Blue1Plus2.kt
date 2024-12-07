package org.firstinspires.ftc.teamcode.opModes.auto.blue.basket.left

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeBeltSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous(name = "Blue 1 + 2 High Basket", group = "Basket", preselectTeleOp = "MainTeleOp")
class Blue1Plus2: OpMode() {
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var intake: Servo
    private lateinit var intakeBelt: Servo

    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var intakeBeltSubsystem: IntakeBeltSubsystem
    private lateinit var armSubsystem: ArmSubsystem
    private lateinit var elevatorSubsystem: ElevatorSubsystem

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        intake = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE.deviceName)
        intakeBelt = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE_BELT.deviceName)

        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intake)
        intakeBeltSubsystem = IntakeBeltSubsystem(intakeBelt)
        armSubsystem = ArmSubsystem(armRight, armLeft)
        elevatorSubsystem = ElevatorSubsystem(elevatorRight, elevatorLeft, armSubsystem::armAngle)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_LEFT.startPose)
            .addTemporalMarker(2.0) {
                intakeSubsystem.intake()
                intakeBeltSubsystem.intakePos()
            }
            .splineToLinearHeading(Pose2d(54.0, 55.0, Math.toRadians(225.0)), Math.toRadians(0.0))
            .back(4.0)
            .addTemporalMarker(1.5) {
                armSubsystem.setpoint = Math.toRadians(98.0)
            }
            .waitSeconds(1.0)
            .addTemporalMarker(4.0) {
                elevatorSubsystem.setpoint = 30.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(4.5) {
                intakeBeltSubsystem.outtakePos()
            }
            .waitSeconds(1.5)
            .addTemporalMarker(6.0) {
                intakeSubsystem.outtake()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(6.5) {
                intakeBeltSubsystem.intakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .turn(Math.toRadians(32.0))
            .waitSeconds(1.0)
            .forward(3.0)
            .addTemporalMarker(7.5) {
                armSubsystem.setpoint = Math.toRadians(0.0)
            }
            .waitSeconds(1.0)
            .addTemporalMarker(8.5) {
                elevatorSubsystem.setpoint = 30.0
            }
            .waitSeconds(1.5)
            .addTemporalMarker(10.0) {
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(1.5)
            .addTemporalMarker(11.5) {
                intakeSubsystem.intake()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(12.0) {
                intakeBeltSubsystem.outtakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .turn(Math.toRadians(-30.0))
            .waitSeconds(1.0)
            .back(2.0)
            .addTemporalMarker(13.0) {
                armSubsystem.setpoint = Math.toRadians(98.0)
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(1.0)
            .addTemporalMarker(15.0) {
                elevatorSubsystem.setpoint = 33.0
            }
            .waitSeconds(2.0)
            .addTemporalMarker(17.0) {
                intakeBeltSubsystem.outtakePos()
            }
            .turn(Math.toRadians(52.0))
            .waitSeconds(1.5)
            .addTemporalMarker(18.5) {
                intakeSubsystem.outtake()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(19.0) {
                intakeBeltSubsystem.intakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(1.5)
            .addTemporalMarker(20.5) {
                intakeBeltSubsystem.outtakePos()
                armSubsystem.setpoint = 0.0
            }
            .waitSeconds(1.0)
            .forward(2.0)
            .addTemporalMarker(21.5) {
                elevatorSubsystem.setpoint = 30.0
                intakeBeltSubsystem.intakePos()
            }
            .turn(Math.toRadians(-60.0))
            .waitSeconds(2.0)
            .addTemporalMarker(23.5) {
                intakeSubsystem.intake()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(24.0) {
                intakeBeltSubsystem.outtakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(0.5)
            .back(2.0)
            .addTemporalMarker(24.5) {
                armSubsystem.setpoint = Math.toRadians(98.0)
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(1.5)
            .addTemporalMarker(27.0) {
                elevatorSubsystem.setpoint = 33.0
            }
            .waitSeconds(1.5)
            .addTemporalMarker(28.5) {
                intakeBeltSubsystem.outtakePos()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(29.0) {
                intakeSubsystem.outtake()
            }
            // .strafetolinearheading

//            .waitSeconds(0.5)
//            .addTemporalMarker(6.0) {
//                intakeBeltSubsystem.outtakePos()
//                armSubsystem.setpoint = 0.0
//            }
//            .turn(Math.toRadians(40.0))
//            .waitSeconds(0.5)
//            .addTemporalMarker(6.5) {
//                elevatorSubsystem.setpoint = 30.0
//            }
//            .waitSeconds(1.0)
//            .addTemporalMarker(7.5) {
//                intakeBeltSubsystem.intakePos()
//            }
//            .waitSeconds(0.5)
//            .addTemporalMarker(8.0) {
//                intakeSubsystem.intake()
//            }
//            .waitSeconds(0.5)
//            .addTemporalMarker(8.5) {
//                elevatorSubsystem.setpoint = 0.0
//                intakeBeltSubsystem.outtakePos()
//            }
//            .turn(Math.toRadians(-40.0))
//            .waitSeconds(1.0)
//            .addTemporalMarker(9.5) {
//                armSubsystem.setpoint = Math.toRadians(98.0)
//            }
            .build()

        driveSubsystem.poseEstimate = AutoStartPose.BLUE_LEFT.startPose
        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.update()

        elevatorSubsystem.periodic()
        armSubsystem.periodic()

        telemetry.addData("Arm Position", armSubsystem.armAngle)
        telemetry.addData("Slides Position", elevatorSubsystem.slidePos)
        telemetry.update()
    }
}