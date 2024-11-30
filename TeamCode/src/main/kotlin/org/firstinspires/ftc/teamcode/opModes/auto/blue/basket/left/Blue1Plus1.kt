package org.firstinspires.ftc.teamcode.opModes.auto.blue.basket.left

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

@Autonomous(name = "Blue 1 + 1 High Basket", group = "Basket", preselectTeleOp = "MainTeleOp")
class Blue1Plus1: OpMode() {
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
            .splineToLinearHeading(Pose2d(55.0, 55.0, Math.toRadians(225.0)), Math.toRadians(0.0))
            .back(4.3)
            .addTemporalMarker(3.0) {
                armSubsystem.setpoint = Math.toRadians(97.5)
            }
            .waitSeconds(1.0)
            .addTemporalMarker(4.0) {
                elevatorSubsystem.setpoint = 2000.0
            }
            .waitSeconds(2.0)
            .addTemporalMarker(7.0) {
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(1.0)
            .addTemporalMarker(8.0) {
                intakeSubsystem.outtake()
            }
            .waitSeconds(1.5)
            .forward(4.0)
            .addTemporalMarker(9.5) {
                elevatorSubsystem.setpoint = 0.0
            }
//            .waitSeconds(2.0)
//            .addTemporalMarker(13.0) {
//                armSubsystem.setpoint = 0.0
//            }
//            .waitSeconds(1.0)
//            .turn(Math.toRadians(25.0))
//            .addTemporalMarker(14.0) {
//                elevatorSubsystem.setpoint = 1500.0
//            }
//            .waitSeconds(2.0)
//            .addTemporalMarker(16.0) {
//                intakeBeltSubsystem.intakePos()
//            }
//            .waitSeconds(1.0)
//            .addTemporalMarker(17.0) {
//                intakeSubsystem.intake()
//            }
//            .waitSeconds(2.0)
//            .addTemporalMarker(20.0) {
//                elevatorSubsystem.setpoint = 100.0
//            }
//            .waitSeconds(2.0)
//            .turn(Math.toRadians(-25.0))
//            .addTemporalMarker(24.0) {
//                armSubsystem.setpoint = 95.0
//                intakeBeltSubsystem.outtakePos()
//
//            }
//            .addTemporalMarker(25.0) {
//                elevatorSubsystem.setpoint = 2000.0
//            }
//            .waitSeconds(2.0)
//            .addTemporalMarker(28.0) {
//                intakeSubsystem.outtake()
//            }
//            .waitSeconds(2.0)
            .lineToSplineHeading(Pose2d(36.0, 8.0, Math.toRadians(180.0)))
            .addTemporalMarker(12.0) {
                armSubsystem.setpoint = 50.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(13.0) {
                elevatorSubsystem.setpoint = 1000.0
            }
            .build()

        driveSubsystem.poseEstimate = AutoStartPose.BLUE_LEFT.startPose
        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.update()

        elevatorSubsystem.periodic()
        armSubsystem.periodic()
    }
}