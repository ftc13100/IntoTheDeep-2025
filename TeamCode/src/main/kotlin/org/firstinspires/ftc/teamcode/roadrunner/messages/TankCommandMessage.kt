package org.firstinspires.ftc.teamcode.roadrunner.messages

class TankCommandMessage(var voltage: Double, var leftPower: Double, var rightPower: Double) {
    var timestamp: Long = System.nanoTime()
}
