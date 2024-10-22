package org.firstinspires.ftc.teamcode.utils.roadrunner.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer

class AprilTagThreeWheelLocalizer(
    private val odometry: StandardTrackingWheelLocalizer,
) : Localizer {
    private var _poseEstimate: Pose2d = Pose2d()

    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            _poseEstimate = value
        }

    override val poseVelocity: Pose2d? = null

    override fun update() {
        // Return a kalman filter that takes the pose estimate from the odometry and april tags to combine
        odometry.update()
        TODO("Not yet implemented")
    }
}