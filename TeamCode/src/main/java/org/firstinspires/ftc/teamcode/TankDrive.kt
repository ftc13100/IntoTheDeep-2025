package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.RamseteController
import com.acmerobotics.roadrunner.TankKinematics
import com.acmerobotics.roadrunner.TankKinematics.WheelIncrements
import com.acmerobotics.roadrunner.TankKinematics.WheelVelocities
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
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.messages.PoseMessage
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage
import java.util.Arrays
import java.util.Collections
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

@Config
class TankDrive(hardwareMap: HardwareMap, var pose: Pose2d) {
    class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP
        var usbFacingDirection: UsbFacingDirection = UsbFacingDirection.FORWARD

        // drive model parameters
        var inPerTick: Double = 0.0
        var trackWidthTicks: Double = 0.0

        // feedforward parameters (in tick units)
        var kS: Double = 0.0
        var kV: Double = 0.0
        var kA: Double = 0.0

        // path profile parameters (in inches)
        var maxWheelVel: Double = 50.0
        var minProfileAccel: Double = -30.0
        var maxProfileAccel: Double = 50.0

        // turn profile parameters (in radians)
        var maxAngVel: Double = Math.PI // shared with path
        var maxAngAccel: Double = Math.PI

        // path controller gains
        var ramseteZeta: Double = 0.7 // in the range (0, 1)
        var ramseteBBar: Double = 2.0 // positive

        // turn controller gains
        var turnGain: Double = 0.0
        var turnVelGain: Double = 0.0
    }

    val kinematics: TankKinematics = TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks)

    val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        Arrays.asList(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    val leftMotors: List<DcMotorEx>
    val rightMotors: List<DcMotorEx>

    val lazyImu: LazyImu

    val voltageSensor: VoltageSensor

    val localizer: Localizer

    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)

    private val tankCommandWriter = DownsampledWriter("TANK_COMMAND", 50000000)

    inner class DriveLocalizer : Localizer {
        lateinit var leftEncs: List<Encoder>
        lateinit var rightEncs: List<Encoder>

        private var lastLeftPos = 0.0
        private var lastRightPos = 0.0
        private var initialized = false

        init {
            run {
                val leftEncs: MutableList<Encoder> = ArrayList()
                for (m in leftMotors) {
                    val e: Encoder = OverflowEncoder(RawEncoder(m))
                    leftEncs.add(e)
                }
                this.leftEncs = Collections.unmodifiableList(leftEncs)
            }

            run {
                val rightEncs: MutableList<Encoder> = ArrayList()
                for (m in rightMotors) {
                    val e: Encoder = OverflowEncoder(RawEncoder(m))
                    rightEncs.add(e)
                }
                this.rightEncs = Collections.unmodifiableList(rightEncs)
            }

            // TODO: reverse encoder directions if needed
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        }

        override fun update(): Twist2dDual<Time> {
            val leftReadings: MutableList<PositionVelocityPair> = ArrayList()
            val rightReadings: MutableList<PositionVelocityPair> = ArrayList()
            var meanLeftPos = 0.0
            var meanLeftVel = 0.0
            for (e in leftEncs) {
                val p = e.getPositionAndVelocity()
                meanLeftPos += p.position.toDouble()
                meanLeftVel += p.velocity.toDouble()
                leftReadings.add(p)
            }
            meanLeftPos /= leftEncs.size.toDouble()
            meanLeftVel /= leftEncs.size.toDouble()

            var meanRightPos = 0.0
            var meanRightVel = 0.0
            for (e in rightEncs) {
                val p = e.getPositionAndVelocity()
                meanRightPos += p.position.toDouble()
                meanRightVel += p.velocity.toDouble()
                rightReadings.add(p)
            }
            meanRightPos /= rightEncs.size.toDouble()
            meanRightVel /= rightEncs.size.toDouble()

            write(
                "TANK_LOCALIZER_INPUTS",
                TankLocalizerInputsMessage(leftReadings, rightReadings)
            )

            if (!initialized) {
                initialized = true

                lastLeftPos = meanLeftPos
                lastRightPos = meanRightPos

                return Twist2dDual(
                    Vector2dDual.constant(Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
                )
            }

            val twist: WheelIncrements<Time> = WheelIncrements(
                DualNum<Time>(
                    doubleArrayOf(
                        meanLeftPos - lastLeftPos,
                        meanLeftVel
                    )
                ).times(PARAMS.inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        meanRightPos - lastRightPos,
                        meanRightVel,
                    )
                ).times(PARAMS.inPerTick)
            )

            lastLeftPos = meanLeftPos
            lastRightPos = meanRightPos

            return kinematics.forward(twist)
        }
    }

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx::class.java, "left"))
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx::class.java, "right"))

        for (m in leftMotors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        for (m in rightMotors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // TODO: reverse motor directions if needed
        //   leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = LazyImu(
            hardwareMap, "imu", RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection
            )
        )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        localizer = DriveLocalizer()

        write("TANK_PARAMS", PARAMS)
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels: WheelVelocities<Time> = TankKinematics(2.0).inverse(
            PoseVelocity2dDual.constant(powers, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        for (m in leftMotors) {
            m.power = wheelVels.left.get(0) / maxPowerMag
        }
        for (m in rightMotors) {
            m.power = wheelVels.right.get(0) / maxPowerMag
        }
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
                for (m in leftMotors) {
                    m.power = 0.0
                }
                for (m in rightMotors) {
                    m.power = 0.0
                }

                return false
            }

            val x = timeTrajectory.profile[t]

            val txWorldTarget = timeTrajectory.path[x.value(), 3]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> =
                RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, pose)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftPower = feedforward.compute(wheelVels.left) / voltage
            val rightPower = feedforward.compute(wheelVels.right) / voltage
            tankCommandWriter.write(TankCommandMessage(voltage, leftPower, rightPower))

            for (m in leftMotors) {
                m.power = leftPower
            }
            for (m in rightMotors) {
                m.power = rightPower
            }

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
                for (m in leftMotors) {
                    m.power = 0.0
                }
                for (m in rightMotors) {
                    m.power = 0.0
                }

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command = PoseVelocity2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 3),
                txWorldTarget.heading.velocity().plus(
                    PARAMS.turnGain * pose.heading.minus(txWorldTarget.heading.value()) +
                            PARAMS.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity()
                        .value())
                )
            )
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftPower = feedforward.compute(wheelVels.left) / voltage
            val rightPower = feedforward.compute(wheelVels.right) / voltage
            tankCommandWriter.write(TankCommandMessage(voltage, leftPower, rightPower))

            for (m in leftMotors) {
                m.power = leftPower
            }
            for (m in rightMotors) {
                m.power = rightPower
            }

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

    fun updatePoseEstimate(): PoseVelocity2d {
        val twist = localizer.update()
        pose = pose.plus(twist.value())

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

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn: TimeTurn -> TurnAction(turn) },
            { t: TimeTrajectory -> FollowTrajectoryAction(t) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }

    companion object {
        var PARAMS: Params = Params()
    }
}
