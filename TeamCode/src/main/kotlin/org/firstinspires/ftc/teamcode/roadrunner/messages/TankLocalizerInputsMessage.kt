package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

class TankLocalizerInputsMessage(
    left: List<PositionVelocityPair>,
    right: List<PositionVelocityPair>
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var left = left.toTypedArray()
    @JvmField var right = right.toTypedArray()
}
