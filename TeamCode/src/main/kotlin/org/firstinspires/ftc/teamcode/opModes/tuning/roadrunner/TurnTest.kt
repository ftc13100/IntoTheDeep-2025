package org.firstinspires.ftc.teamcode.opModes.tuning.roadrunner

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

/*
 * This is a simple routine to test turning capabilities.
 */
//@Disabled
@Config
@Autonomous(group = "drive")
class TurnTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = DriveSubsystem(hardwareMap)
        waitForStart()
        if (isStopRequested) return
        drive.turn(Math.toRadians(ANGLE))
    }

    companion object {
        @JvmField
        var ANGLE = 180.0 // deg
    }
}