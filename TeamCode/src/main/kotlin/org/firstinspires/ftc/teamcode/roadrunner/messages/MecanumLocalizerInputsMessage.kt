package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class MecanumLocalizerInputsMessage(
    @JvmField var leftFront: PositionVelocityPair,
    @JvmField var leftBack: PositionVelocityPair,
    @JvmField var rightBack: PositionVelocityPair,
    @JvmField var rightFront: PositionVelocityPair,
    angles: YawPitchRollAngles
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var yaw: Double = angles.getYaw(AngleUnit.RADIANS)
    @JvmField var pitch: Double = angles.getPitch(AngleUnit.RADIANS)
    @JvmField var roll: Double = angles.getRoll(AngleUnit.RADIANS)
}
