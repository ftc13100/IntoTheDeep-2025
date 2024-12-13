package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.roadrunner.Pose2d

enum class AutoStartPose(val startPose: Pose2d) {
    BLUE_LEFT(
        Pose2d(
            38.0, 62.0, Math.toRadians(270.0)
        )
    ),

    BLUE_RIGHT(
        Pose2d(
            12.0, 62.0, Math.toRadians(270.0)
        )
    ),

    RED_LEFT(
        Pose2d(
            -12.0, -62.0, Math.toRadians(90.0)
        )
    ),

    RED_RIGHT(
        Pose2d(
            12.0, -62.0, Math.toRadians(90.0)
        )
    ),
}