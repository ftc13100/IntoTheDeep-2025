package org.firstinspires.ftc.teamcode.opModes.auto.blue.basket.right


import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.slides.ElevatorSubsystem


@Autonomous(name = "Blue High Basket Right", group = "Basket", preselectTeleOp = "MainTeleOp")
class BlueHighBasketRight : OpMode() {
    private lateinit var armLeft: Motor
    private lateinit var armRight: Motor
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var intake: Servo

    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var armSubsystem: ArmSubsystem
    private lateinit var elevatorSubsystem: ElevatorSubsystem

    override fun init() {
        intake = hardwareMap.get(Servo::class.java, ControlBoard.INTAKE.deviceName)

        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)

        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intake)
        armSubsystem = ArmSubsystem(armRight, armLeft)
        elevatorSubsystem = ElevatorSubsystem(elevatorRight, elevatorLeft, armSubsystem::armAngle)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_RIGHT.startPose)
            .turn(Math.toRadians(90.0))
            .forward(35.0)
            .addTemporalMarker(2.0) {
                armSubsystem.setpoint = Math.toRadians(90.0)
            }
            .waitSeconds(1.0)
            .addTemporalMarker(3.0) {
                elevatorSubsystem.setpoint = 200.0
            }
            .waitSeconds(2.0)
            .addTemporalMarker(5.0){
                intakeSubsystem.outtake()
            }
            .waitSeconds(1.0)
            .addTemporalMarker(6.0) {
                elevatorSubsystem.setpoint = 0.0
            }
            .waitSeconds(2.0)
            .addTemporalMarker(8.0) {
                armSubsystem.setpoint = Math.toRadians(0.0)
            }
            .build()
        driveSubsystem.poseEstimate = AutoStartPose.BLUE_RIGHT.startPose


        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.update()
    }
}