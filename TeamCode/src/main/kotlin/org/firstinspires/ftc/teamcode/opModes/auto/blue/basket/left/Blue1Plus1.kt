package org.firstinspires.ftc.teamcode.opModes.auto.blue.basket.left

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem

@Autonomous(name = "Blue 1 + 3 High Basket", group = "Basket", preselectTeleOp = "MainTeleOp")
class Blue1Plus1: OpMode() {
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var intake: CRServo

    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var armSubsystem: ArmSubsystem
    private lateinit var elevatorSubsystem: ElevatorSubsystem

    override fun init() {
//        intake = CRServo(hardwareMap, ControlBoard.INTAKE.deviceName)

        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
//        intakeSubsystem = IntakeSubsystem(intake)
        armSubsystem = ArmSubsystem(armRight, armLeft)
        elevatorSubsystem = ElevatorSubsystem(elevatorRight, elevatorLeft, armSubsystem::armAngle)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_LEFT.startPose)
            .addTemporalMarker(0.1) {
                armSubsystem.setpoint = Math.toRadians(95.0)
            }
            .waitSeconds(0.25)
            .splineToLinearHeading(Pose2d(55.0, 55.0, Math.toRadians(225.0)), Math.toRadians(0.0))
            .waitSeconds(0.25)
            .addTemporalMarker(3.0) {
                elevatorSubsystem.setpoint = 1700.0
                //turn belt servo
            }
            //outtake
            .waitSeconds(1.0)
            .turn(Math.toRadians(30.0))
            .addTemporalMarker(7.0) {
                elevatorSubsystem.setpoint = 0.0
                //belt servo turn
            }
//            .addTemporalMarker(8.0) {
//                armSubsystem.setpoint = Math.toRadians(0.0)
//            }
//            .addTemporalMarker(9.0) {
//                elevatorSubsystem.setpoint = 1700.0
//                //intake
//            }
//            .addTemporalMarker(12.0) {
//                elevatorSubsystem.setpoint = 0.0
//            }
//            .addTemporalMarker(15.0) {
//                armSubsystem.setpoint = 95.0
//                //turn belt servo
//            }
//            .addTemporalMarker(18.0) {
//                //outtake
//            }
//            .waitSeconds(2.0)
//            .splineToSplineHeading(Pose2d(36.0, 12.0, Math.toRadians(-180.0)), Math.toRadians(-90.0))
//            .turn(Math.toRadians(-30.0))
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