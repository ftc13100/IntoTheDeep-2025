package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object DriveSubsystem : SubsystemBase() {
    private lateinit var hardwareMap: HardwareMap
    lateinit var drive: MecanumDrive

    var pose: Pose2d = drive.pose
        set(value) {
            drive.pose = value
            field = value
        }
    
    var driveMultiplier = 0.9

    fun initialize(hardwareMap: HardwareMap, reset: Boolean = true) {
        if (reset)
            drive.pose = Pose2d(0.0, 0.0, 0.0)

        this.hardwareMap = hardwareMap
        drive = PinpointDrive(hardwareMap)

        write("MECANUM_drive.PARAMS", MecanumDrive.PARAMS)
    }

    fun actionBuilder(beginPose: () -> Pose2d) = drive.actionBuilder(beginPose)

    fun drive(leftY: Double, leftX: Double, rightX: Double) {
        drive.setDrivePowers(
            PoseVelocity2d(
                linearVel = Vector2d(leftY * driveMultiplier, -leftX * driveMultiplier),
                angVel = -rightX * driveMultiplier
            )
        )
    }
}