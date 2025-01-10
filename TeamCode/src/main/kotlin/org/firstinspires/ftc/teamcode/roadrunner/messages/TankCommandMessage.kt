package org.firstinspires.ftc.teamcode.roadrunner.messages

class TankCommandMessage(
    @JvmField var voltage: Double,
    @JvmField var leftPower: Double,
    @JvmField var rightPower: Double
) {
    @JvmField var timestamp: Long = System.nanoTime()
}
