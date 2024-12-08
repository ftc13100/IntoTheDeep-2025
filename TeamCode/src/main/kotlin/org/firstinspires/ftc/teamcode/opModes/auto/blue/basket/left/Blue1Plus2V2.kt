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
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequence

@Autonomous(name = "V2 Blue 1 + 2", group = "Basket", preselectTeleOp = "MainTeleOp")
class Blue1Plus2V2: OpMode() {
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

    private lateinit var toBasket1: TrajectorySequence
    private lateinit var toBasket2: TrajectorySequence

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

        toBasket1 = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_LEFT.startPose)
            .splineToLinearHeading(Pose2d(54.0, 56.0, Math.toRadians(225.0)), Math.toRadians(0.0))
            .addTemporalMarker(0.1) {
                intakeSubsystem.intake()
                armSubsystem.setpoint = Math.toRadians(98.0)
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(1.0) {
                elevatorSubsystem.setpoint = 34.0
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(1.5) {
                intakeBeltSubsystem.outtakePos()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(2.0) {
                intakeSubsystem.outtake()
            }
            .waitSeconds(0.1)
            .addTemporalMarker(2.1) {
                intakeBeltSubsystem.outtakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .turn(Math.toRadians(37.0))
            .waitSeconds(2.0)
            .addTemporalMarker(4.1) {
                armSubsystem.setpoint = Math.toRadians(0.0)
            }
            .waitSeconds(0.5)
            .addTemporalMarker(4.6) {
                elevatorSubsystem.setpoint = 19.0
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(1.25)
            .addTemporalMarker(6.0) {
                intakeSubsystem.intake()
            }
            .waitSeconds(0.25)
            .turn(Math.toRadians(-37.0))
            .addTemporalMarker(6.25) {
                intakeBeltSubsystem.outtakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(7.05) {
                armSubsystem.setpoint = Math.toRadians(98.0)
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(0.75)
            .addTemporalMarker(7.8) {
                elevatorSubsystem.setpoint = 31.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(8.8) {
                intakeBeltSubsystem.outtakePos()
            }
            .waitSeconds(1.25)
            .turn(Math.toRadians(55.0))
            .addTemporalMarker(10.05) {
                intakeSubsystem.outtake()
            }
            .waitSeconds(0.25)
            .addTemporalMarker(10.3) {
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(10.8) {
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(1.5)
            .addTemporalMarker(12.3) {
                armSubsystem.setpoint = 0.0
                intakeBeltSubsystem.outtakePos()
            }
            .waitSeconds(1.0)
            .addTemporalMarker(13.3) {
                elevatorSubsystem.setpoint = 19.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(14.3) {
                intakeBeltSubsystem.intakePos()
            }
            .turn(Math.toRadians(-55.0))
            .waitSeconds(1.2)
            .back(2.0)
            .addTemporalMarker(15.5) {
                intakeSubsystem.intake()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(16.0) {
                intakeBeltSubsystem.outtakePos()
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(17.0) {
                armSubsystem.setpoint = Math.toRadians(98.0)
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(1.0)
            .addTemporalMarker(18.0) {
                elevatorSubsystem.setpoint = 29.0
            }
            .waitSeconds(1.5)
            .addTemporalMarker(19.5) {
                intakeBeltSubsystem.outtakePos()
            }
            .waitSeconds(1.0)
            .addTemporalMarker(20.5) {
                intakeSubsystem.outtake()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(21.0) {
                intakeBeltSubsystem.intakePos()
            }
            .waitSeconds(0.5)
            .addTemporalMarker(21.5) {
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(1.0)
            .addTemporalMarker(22.5) {
                armSubsystem.setpoint = 0.0
            }
            .splineToSplineHeading(Pose2d(28.0, 10.0, Math.toRadians(0.0)), 90.0)
            .back(5.0)
            .addDisplacementMarker {
                driveSubsystem.followTrajectorySequenceAsync(toBasket2)
            }
            .build()
        driveSubsystem.poseEstimate = AutoStartPose.BLUE_LEFT.startPose
        driveSubsystem.followTrajectorySequenceAsync(toBasket1)
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