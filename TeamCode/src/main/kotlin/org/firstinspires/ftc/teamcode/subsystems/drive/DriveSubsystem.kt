package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.Angle
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.Subsystem
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.StandardTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.utils.roadrunner.util.LynxModuleUtil
import kotlin.math.PI
import kotlin.math.abs

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
class DriveSubsystem @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    private val fieldCentric: Boolean = false,
) : MecanumDrive(
    DriveConstants.kV,
    DriveConstants.kA,
    DriveConstants.kStatic,
    DriveConstants.TRACK_WIDTH,
    DriveConstants.TRACK_WIDTH,
    LATERAL_MULTIPLIER
), Subsystem {
    private val trajectorySequenceRunner: TrajectorySequenceRunner
    private val leftFront: DcMotorEx
    private val leftRear: DcMotorEx
    private val rightRear: DcMotorEx
    private val rightFront: DcMotorEx
    private val motors: List<DcMotorEx>
    private val batteryVoltageSensor: VoltageSensor

    var m_name = this.javaClass.simpleName

    init {
        val follower: TrajectoryFollower = HolonomicPIDVAFollower(
            TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        leftFront = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_LEFT_FRONT.deviceName)
        leftRear = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_LEFT_REAR.deviceName)
        rightRear = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_RIGHT_REAR.deviceName)
        rightFront = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_RIGHT_FRONT.deviceName)
        motors = listOf(leftFront, leftRear, rightRear, rightFront)
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        if (DriveConstants.RUN_USING_ENCODER) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID)
        }

        //leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE

        localizer = StandardTrackingWheelLocalizer(hardwareMap)

        trajectorySequenceRunner = TrajectorySequenceRunner(follower, HEADING_PID)

        CommandScheduler.getInstance().registerSubsystem(this)
    }

    @JvmOverloads
    fun trajectoryBuilder(
        startPose: Pose2d,
        reversed: Boolean = false,
        startHeading: Double = startPose.heading,
    ): TrajectoryBuilder {
        return TrajectoryBuilder(
            startPose,
            Angle.norm(startHeading + if (reversed) PI else 0.0),
            VEL_CONSTRAINT,
            ACCEL_CONSTRAINT
        )
    }

    fun drive(leftY: Double, leftX: Double, rightX: Double) {
        val (_, _, heading) = poseEstimate

        val (x, y) = Vector2d(
            -leftY,
            -leftX
        ).rotated(
            if (fieldCentric) -heading else 0.0
        )

        setWeightedDrivePower(Pose2d(x, y, -rightX))

        update()
        PoseStorage.poseEstimate = poseEstimate
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose,
            VEL_CONSTRAINT, ACCEL_CONSTRAINT,
            DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        )
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(poseEstimate)
                .turn(angle)
                .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    @Suppress("unused", "MemberVisibilityCanBePrivate")
    val lastError: Pose2d
        get() = trajectorySequenceRunner.lastPoseError

    fun update() {
        updatePoseEstimate()
        val signal = trajectorySequenceRunner.update(poseEstimate, poseVelocity)
        signal?.let { setDriveSignal(it) }
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) update()
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunner.isBusy

    fun setMode(runMode: DcMotor.RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower
        if ((abs(drivePower.x) + abs(drivePower.y)
                    + abs(drivePower.heading)) > 1
        ) {
            // re-normalize the powers according to the weights
            val denom =
                VX_WEIGHT * abs(drivePower.x) + VY_WEIGHT * abs(drivePower.y) + OMEGA_WEIGHT * abs(
                    drivePower.heading
                )
            vel = Pose2d(
                VX_WEIGHT * drivePower.x,
                VY_WEIGHT * drivePower.y,
                OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }
        setDrivePower(vel)
    }

    override fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    @Suppress("unused")
    override fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelVelocities.add(DriveConstants.encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    override fun setMotorPowers(
        frontLeft: Double,
        rearLeft: Double,
        rearRight: Double,
        frontRight: Double,
    ) {
        leftFront.power = frontLeft
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
    }

    override val rawExternalHeading: Double
        get() = 0.0

    override fun getExternalHeadingVelocity() = 0.0

    companion object {
        var TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 0.0)
        var HEADING_PID = PIDCoefficients(8.0, 0.0, 0.0)
        var LATERAL_MULTIPLIER = 1.0126582278481012658227848101266
        var VX_WEIGHT = 1.0
        var VY_WEIGHT = 1.0
        var OMEGA_WEIGHT = 1.0
        private val VEL_CONSTRAINT = getVelocityConstraint(
            DriveConstants.MAX_VEL,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.TRACK_WIDTH
        )
        private val ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL)
        fun getVelocityConstraint(
            maxVel: Double,
            maxAngularVel: Double,
            trackWidth: Double,
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(maxAngularVel),
                    MecanumVelocityConstraint(maxVel, trackWidth)
                )
            )
        }

        fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }
}