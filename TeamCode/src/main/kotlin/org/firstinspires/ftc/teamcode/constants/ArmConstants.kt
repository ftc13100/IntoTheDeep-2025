package org.firstinspires.ftc.teamcode.constants

enum class ArmConstants(val value: Double) {
    kP(5.0),
    kI(0.0),
    kD(0.1),
    kS(0.002),
    kCos(0.07),
    kV(0.32),

    MAX_VELOCITY(3.0), // rad/s
    MAX_ACCELERATION(36.0) // rad/s^2
}