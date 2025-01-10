package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

data class ThreeDeadWheelInputsMessage(
    @JvmField var par0: PositionVelocityPair,
    @JvmField var par1: PositionVelocityPair,
    @JvmField var perp: PositionVelocityPair
) {
    @JvmField var timestamp: Long = System.nanoTime()
}