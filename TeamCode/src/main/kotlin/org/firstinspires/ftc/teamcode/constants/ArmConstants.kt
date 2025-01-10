package org.firstinspires.ftc.teamcode.constants

enum class ArmConstants(val value: Double) {
    kP(3.0),
    kI(0.0),
    kD(0.2),

    kS(0.002),
    kCos(0.05),
    kV(0.29),

    MAX_VELOCITY(3.0), // rad/s
    MAX_ACCELERATION(40.0) // rad/s^2
}