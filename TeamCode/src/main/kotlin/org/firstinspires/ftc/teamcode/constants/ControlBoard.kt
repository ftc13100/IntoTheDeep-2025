package org.firstinspires.ftc.teamcode.constants


enum class ControlBoard(val deviceName: String) {
    // Drive motors
    DRIVE_LEFT_FRONT("leftFront"),
    DRIVE_RIGHT_FRONT("rightFront"),
    DRIVE_LEFT_REAR("leftRear"),
    DRIVE_RIGHT_REAR("rightRear"),

    // Odometry
    ODO_LEFT_ENCODER("rightSlideString"),
    ODO_RIGHT_ENCODER("rightFront"),
    ODO_STRAFE_ENCODER("rightSlideAxel"),

    // Arm
    ARM_LEFT("leftSlideAxel"),
    ARM_RIGHT("rightSlideAxel"),
    SLIDES_LEFT("leftSlideString"),
    SLIDES_RIGHT("rightSlideString"),

    //Intake
    INTAKE("intakeRoller"),
    INTAKE_BELT("intakeBelt"),
    INTAKE_ENCODER("leftFront"),
    INTAKE_TOUCH("intakeTouch"),
}
