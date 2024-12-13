package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

class TankLocalizerInputsMessage(
    left: List<PositionVelocityPair>,
    right: List<PositionVelocityPair>
) {
    var timestamp: Long = System.nanoTime()
    var left: Array<PositionVelocityPair> = left.toTypedArray<PositionVelocityPair>()
    var right: Array<PositionVelocityPair> = right.toTypedArray<PositionVelocityPair>()
}
