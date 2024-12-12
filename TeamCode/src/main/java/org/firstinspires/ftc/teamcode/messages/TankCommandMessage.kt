package org.firstinspires.ftc.teamcode.messages

class TankCommandMessage(var voltage: Double, var leftPower: Double, var rightPower: Double) {
    var timestamp: Long = System.nanoTime()
}
