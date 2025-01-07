package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MecanumKinematics.WheelIncrements
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.Localizer
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

open class MecanumDrive(
    hardwareMap: HardwareMap,
    var pose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) {
    val leftFront: DcMotorEx
    val leftRear: DcMotorEx
    val rightRear: DcMotorEx
    val rightFront: DcMotorEx

    val voltageSensor: VoltageSensor

    var lazyImu: LazyImu

    val localizer: Localizer

    val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_LEFT_FRONT.deviceName)
        leftRear = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_LEFT_REAR.deviceName)
        rightRear = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_RIGHT_REAR.deviceName)
        rightFront = hardwareMap.get(DcMotorEx::class.java, ControlBoard.DRIVE_RIGHT_FRONT.deviceName)

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftRear.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightRear.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        leftRear.direction = DcMotorSimple.Direction.REVERSE;

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = LazyImu(
            hardwareMap, "imu", RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection
            )
        )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        localizer = DriveLocalizer()

    }

    class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        var logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN
        var usbFacingDirection = UsbFacingDirection.LEFT

        // drive model parameters
        @JvmField var inPerTick = 0.00297208937
        @JvmField var lateralInPerTick = 0.0018615669110367
        @JvmField var trackWidthTicks = 4670.40351221378

        // feedforward parameters (in tick units)
        @JvmField var kS = 1.566240346045443
        @JvmField var kV = 0.0004
        @JvmField var kA = 0.0001

        // path profile parameters (in inches)
        @JvmField var maxWheelVel = 50.0
        @JvmField var minProfileAccel = -30.0
        @JvmField var maxProfileAccel = 50.0

        // turn profile parameters (in radians)
        @JvmField var maxAngVel = Math.PI // shared with path
        @JvmField var maxAngAccel = Math.PI

        // path controller gains
        @JvmField var axialGain = 8.0
        @JvmField var lateralGain = 8.0
        @JvmField var headingGain = 9.0 // shared with turn

        @JvmField var axialVelGain = 0.0
        @JvmField var lateralVelGain = 0.0
        @JvmField var headingVelGain = 0.0 // shared with turn
    }

    val kinematics: MecanumKinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick
    )

    val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    inner class DriveLocalizer : Localizer {
        val leftFront: Encoder =
            OverflowEncoder(RawEncoder(this@MecanumDrive.leftFront))
        val leftRear: Encoder =
            OverflowEncoder(RawEncoder(this@MecanumDrive.leftRear))
        val rightRear: Encoder =
            OverflowEncoder(RawEncoder(this@MecanumDrive.rightRear))
        val rightFront: Encoder =
            OverflowEncoder(RawEncoder(this@MecanumDrive.rightFront))

        val imu: IMU = lazyImu.get()

        private var lastLeftFrontPos = 0.0
        private var lastLeftBackPos = 0.0
        private var lastRightBackPos = 0.0
        private var lastRightFrontPos = 0.0

        private var lastHeading = Rotation2d.exp(imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

        private var initialized = false

//        init {
        // TODO: reverse encoders if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        }

        override fun update(): Twist2dDual<Time> {
            val leftFrontPosVel = leftFront.getPositionAndVelocity()
            val leftBackPosVel = leftRear.getPositionAndVelocity()
            val rightBackPosVel = rightRear.getPositionAndVelocity()
            val rightFrontPosVel = rightFront.getPositionAndVelocity()

            val angles = imu.robotYawPitchRollAngles

            write(
                "MECANUM_LOCALIZER_INPUTS", MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles
                )
            )

            val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

            if (!initialized) {
                initialized = true

                lastLeftFrontPos = leftFrontPosVel.position
                lastLeftBackPos = leftBackPosVel.position
                lastRightBackPos = rightBackPosVel.position
                lastRightFrontPos = rightFrontPosVel.position

                lastHeading = heading

                return Twist2dDual(
                    Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
                )
            }

            val headingDelta = heading - lastHeading
            val twist = kinematics.forward(
                WheelIncrements(
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                            leftFrontPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftBackPosVel.position - lastLeftBackPos).toDouble(),
                            leftBackPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightBackPosVel.position - lastRightBackPos).toDouble(),
                            rightBackPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                            rightFrontPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick)
                )
            )

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            return Twist2dDual(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
            )
        }
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels: MecanumKinematics.WheelVelocities<Time> = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual.constant(powers, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftRear.power = wheelVels.leftBack[0] / maxPowerMag
        rightRear.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0, timeTrajectory.path.length(),
                max(
                    2.0,
                    (ceil(timeTrajectory.path.length() / 2).toInt()).toDouble()
                ).toInt()
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftRear.power = 0.0
                rightRear.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = timeTrajectory[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, pose, robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = voltageSensor.voltage

            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = leftFrontPower
            leftRear.power = leftBackPower
            rightRear.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", pose.position.x)
            p.put("y", pose.position.y)
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()))

            val error = txWorldTarget.value().minusExp(pose)
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, pose)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftRear.power = 0.0
                rightRear.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, pose, robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftRear.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightRear.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, pose)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    open fun updatePoseEstimate(): PoseVelocity2d {
        val twist = localizer.update()
        pose += twist.value()

        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(pose))

        return twist.velocity().value()
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for ((position) in poseHistory) {
            xPoints[i] = position.x
            yPoints[i] = position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: () -> Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { TurnAction(it) },
            { FollowTrajectoryAction(it) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose.invoke(), 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }
    
    companion object {
        @JvmField var PARAMS = Params()
    }
}