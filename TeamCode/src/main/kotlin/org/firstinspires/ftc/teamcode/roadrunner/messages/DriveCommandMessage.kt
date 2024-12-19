package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time

class DriveCommandMessage(poseVelocity: PoseVelocity2dDual<Time>) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var forwardVelocity: Double = poseVelocity.linearVel.x[0]
    @JvmField var forwardAcceleration: Double = poseVelocity.linearVel.x[1]
    @JvmField var lateralVelocity: Double = poseVelocity.linearVel.y[0]
    @JvmField var lateralAcceleration: Double = poseVelocity.linearVel.y[1]
    @JvmField var angularVelocity: Double = poseVelocity.angVel[0]
    @JvmField var angularAcceleration: Double = poseVelocity.angVel[1]
}
