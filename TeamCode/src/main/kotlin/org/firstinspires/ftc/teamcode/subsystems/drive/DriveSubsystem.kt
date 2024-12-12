package org.firstinspires.ftc.teamcode.subsystems.drive

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap

object DriveSubsystem : SubsystemBase() {
    private lateinit var hardwareMap: HardwareMap

    fun initialize(hardwareMap: HardwareMap) {
        this.hardwareMap = hardwareMap
    }

    fun drive(leftY: Double, leftX: Double, rightX: Double) = 0.0
}