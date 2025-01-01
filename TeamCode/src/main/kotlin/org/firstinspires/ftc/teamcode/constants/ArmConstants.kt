package org.firstinspires.ftc.teamcode.constants

enum class ArmConstants(val value: Double) {
    kP(3.0),
    kI(0.0),
    kD(0.0),
    kCos(0.07),

    MAX_VELOCITY(3.0), // rad/s
    MAX_ACCELERATION(36.0) // rad/s^2
}