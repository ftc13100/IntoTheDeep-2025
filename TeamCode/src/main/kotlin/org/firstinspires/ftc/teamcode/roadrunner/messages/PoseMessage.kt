package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.Pose2d

class PoseMessage(pose: Pose2d) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var x: Double = pose.position.x
    @JvmField var y: Double = pose.position.y
    @JvmField var heading: Double = pose.heading.toDouble()
}

