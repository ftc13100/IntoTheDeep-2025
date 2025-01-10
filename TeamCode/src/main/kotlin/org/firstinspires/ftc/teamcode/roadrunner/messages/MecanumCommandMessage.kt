package org.firstinspires.ftc.teamcode.roadrunner.messages

class MecanumCommandMessage(
    @JvmField var voltage: Double,
    @JvmField var leftFrontPower: Double,
    @JvmField var leftBackPower: Double,
    @JvmField var rightBackPower: Double,
    @JvmField var rightFrontPower: Double
) {
    @JvmField var timestamp: Long = System.nanoTime()
}
