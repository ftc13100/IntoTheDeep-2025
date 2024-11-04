package org.firstinspires.ftc.teamcode.constants


enum class ControlBoard(val deviceName: String) {
    // Drive motors
    DRIVE_LEFT_FRONT("leftFront"),
    DRIVE_RIGHT_FRONT("rightFront"),
    DRIVE_LEFT_REAR("leftRear"),
    DRIVE_RIGHT_REAR("rightRear"),

    // Odometry
    ODO_LEFT_ENCODER("leftFront"),
    ODO_RIGHT_ENCODER("rightFront"),
    ODO_STRAFE_ENCODER("leftSlideString"),

    // Arm
    ARM_LEFT(""),
    ARM_RIGHT(""),
    SLIDES_LEFT(""),
    SLIDES_RIGHT(""),

    // Camera
    CAMERA("lifecam")
}
