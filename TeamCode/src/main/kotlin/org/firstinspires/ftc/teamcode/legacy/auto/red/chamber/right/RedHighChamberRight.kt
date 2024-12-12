package org.firstinspires.ftc.teamcode.legacy.auto.red.chamber.right

//@Autonomous(name = "Red High Chamber Right", group = "Chamber", preselectTeleOp = "MainTeleOp")
//class RedHighChamberRight : OpMode() {
//    private lateinit var armLeft: Motor
//    private lateinit var armRight: Motor
//    private lateinit var elevatorLeft: Motor
//    private lateinit var elevatorRight: Motor
//
//    private lateinit var intake: CRServo
//
//    private lateinit var driveSubsystem: DriveSubsystem
//    private lateinit var intakeSubsystem: IntakeSubsystem
//    private lateinit var armSubsystem: ArmSubsystem
//    private lateinit var elevatorSubsystem: SlidesSubsytem
//
//    override fun init() {
//        intake = CRServo(hardwareMap, ControlBoard.INTAKE.deviceName)
//
//        armLeft = Motor(hardwareMap, ControlBoard.ARM_LEFT.deviceName)
//        armRight = Motor(hardwareMap, ControlBoard.ARM_RIGHT.deviceName)
//
//        elevatorLeft = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
//        elevatorRight = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)
//
//        driveSubsystem = DriveSubsystem(hardwareMap)
//        intakeSubsystem = IntakeSubsystem(intake)
//        armSubsystem = ArmSubsystem(armRight, armLeft)
//        elevatorSubsystem = SlidesSubsytem(elevatorRight, elevatorLeft)
//
//        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_RIGHT.startPose)
//
//            .forward(28.0)
//            .strafeLeft(11.0)
//            .addTemporalMarker(2.0) {
//                armSubsystem.setpoint = Math.toRadians(150.0)
//            }
//            .waitSeconds(1.0)
//            .addTemporalMarker(3.0) {
//                elevatorSubsystem.setpoint = 2.0
//            }
//            .waitSeconds(2.0)
//            .addTemporalMarker(5.0) {
//                intakeSubsystem.outtake()
//            }
//            .waitSeconds(1.0)
//            .addTemporalMarker(6.0) {
//                elevatorSubsystem.setpoint = 0.0
//            }
//            .waitSeconds(2.0)
//            .addTemporalMarker(8.0) {
//                armSubsystem.setpoint = Math.toRadians(0.0)
//            }
//            .build()
//        driveSubsystem.poseEstimate = AutoStartPose.RED_RIGHT.startPose
//
//
//        driveSubsystem.followTrajectorySequenceAsync(trajectory)
//    }
//
//    override fun loop() {
//        driveSubsystem.update()
//    }
//}