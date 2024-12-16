package org.firstinspires.ftc.teamcode.constants

enum class SlidesConstants(val value: Double) {
    kP(0.4),
    kI(0.0),
    kD(0.0),
    kG(0.1),

    MAX_VELOCITY(36.0), // in/s
    MAX_ACCELERATION(420.0), // in/s^2

    MAX_HEIGHT_TICKS(2081.0),
    MAX_HEIGHT_INCH(33.329)
}