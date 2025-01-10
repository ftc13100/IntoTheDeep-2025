package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.PinpointDrive

class LocalizationTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        when (TuningOpModes.DRIVE_CLASS) {
            PinpointDrive::class.java -> {
                DriveSubsystem.initialize(hardwareMap)

                val drive = DriveSubsystem.drive

                waitForStart()

                while (opModeIsActive()) {
                    drive.setDrivePowers(
                        PoseVelocity2d(
                            Vector2d(
                                -gamepad1.left_stick_y.toDouble(),
                                -gamepad1.left_stick_x.toDouble()
                            ),
                            -gamepad1.right_stick_x.toDouble()
                        )
                    )

                    drive.updatePoseEstimate()

                    telemetry.addData("x", drive.pose.position.x)
                    telemetry.addData("y", drive.pose.position.y)
                    telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()))
                    telemetry.update()

                    val packet = TelemetryPacket()
                    packet.fieldOverlay().setStroke("#3F51B5")
                    Drawing.drawRobot(packet.fieldOverlay(), drive.pose)
                    FtcDashboard.getInstance().sendTelemetryPacket(packet)
                }
            }

            else -> {
                throw RuntimeException()
            }
        }
    }
}
