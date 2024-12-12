package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.ForwardPushTest
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger
import com.acmerobotics.roadrunner.ftc.LateralPushTest
import com.acmerobotics.roadrunner.ftc.LateralRampLogger
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer
import java.util.Arrays

object TuningOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    val DRIVE_CLASS: Class<*> = MecanumDrive::class.java

    const val GROUP: String = "quickstart"
    const val DISABLED: Boolean = false

    private fun metaForClass(cls: Class<out OpMode?>): OpModeMeta {
        return OpModeMeta.Builder()
            .setName(cls.simpleName)
            .setGroup(GROUP)
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .build()
    }

    @OpModeRegistrar
    fun register(manager: OpModeManager) {
        if (DISABLED) return

        val dvf: DriveViewFactory
        when (DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                dvf = object : DriveViewFactory {
                    override fun make(h: HardwareMap): DriveView {
                        val md = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                        val leftEncs: MutableList<Encoder> = ArrayList()
                        val rightEncs: MutableList<Encoder> = ArrayList()
                        val parEncs: MutableList<Encoder> = ArrayList()
                        val perpEncs: MutableList<Encoder> = ArrayList()
                        if (md.localizer is MecanumDrive.DriveLocalizer) {
                            val dl = md.localizer
                            leftEncs.add(dl.leftFront)
                            leftEncs.add(dl.leftBack)
                            rightEncs.add(dl.rightFront)
                            rightEncs.add(dl.rightBack)
                        } else if (md.localizer is ThreeDeadWheelLocalizer) {
                            val dl = md.localizer
                            parEncs.add(dl.par0)
                            parEncs.add(dl.par1)
                            perpEncs.add(dl.perp)
                        } else if (md.localizer is TwoDeadWheelLocalizer) {
                            val dl = md.localizer
                            parEncs.add(dl.par)
                            perpEncs.add(dl.perp)
                        } else {
                            throw RuntimeException("unknown localizer: " + md.localizer.javaClass.name)
                        }
                        return DriveView(
                            DriveType.MECANUM,
                            MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.maxWheelVel,
                            MecanumDrive.PARAMS.minProfileAccel,
                            MecanumDrive.PARAMS.maxProfileAccel,
                            hardwareMap.getAll(LynxModule::class.java),
                            listOf(
                                md.leftFront,
                                md.leftBack
                            ),
                            listOf(
                                md.rightFront,
                                md.rightBack
                            ),
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            md.lazyImu,
                            md.voltageSensor
                        ) {
                            MotorFeedforward(
                                MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick
                            )
                        }
                    }
                }
            }
            TankDrive::class.java -> {
                dvf = object : DriveViewFactory {
                    override fun make(h: HardwareMap): DriveView {
                        val td = TankDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                        val leftEncs: MutableList<Encoder> = ArrayList()
                        val rightEncs: MutableList<Encoder> = ArrayList()
                        val parEncs: MutableList<Encoder> = ArrayList()
                        val perpEncs: MutableList<Encoder> = ArrayList()
                        when (td.localizer) {
                            is TankDrive.DriveLocalizer -> {
                                val dl = td.localizer
                                leftEncs.addAll(dl.leftEncs)
                                rightEncs.addAll(dl.rightEncs)
                            }

                            is ThreeDeadWheelLocalizer -> {
                                val dl = td.localizer
                                parEncs.add(dl.par0)
                                parEncs.add(dl.par1)
                                perpEncs.add(dl.perp)
                            }

                            is TwoDeadWheelLocalizer -> {
                                val dl = td.localizer
                                parEncs.add(dl.par)
                                perpEncs.add(dl.perp)
                            }

                            else -> {
                                throw RuntimeException("unknown localizer: " + td.localizer.javaClass.name)
                            }
                        }
                        return DriveView(
                            DriveType.TANK,
                            TankDrive.PARAMS.inPerTick,
                            TankDrive.PARAMS.maxWheelVel,
                            TankDrive.PARAMS.minProfileAccel,
                            TankDrive.PARAMS.maxProfileAccel,
                            hardwareMap.getAll(LynxModule::class.java),
                            td.leftMotors,
                            td.rightMotors,
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            td.lazyImu,
                            td.voltageSensor
                        ) {
                            MotorFeedforward(
                                TankDrive.PARAMS.kS,
                                TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick,
                                TankDrive.PARAMS.kA / TankDrive.PARAMS.inPerTick
                            )
                        }
                    }
                }
            }
            else -> {
                throw RuntimeException()
            }
        }

        manager.register(metaForClass(AngularRampLogger::class.java), AngularRampLogger(dvf))
        manager.register(metaForClass(ForwardPushTest::class.java), ForwardPushTest(dvf))
        manager.register(metaForClass(ForwardRampLogger::class.java), ForwardRampLogger(dvf))
        manager.register(metaForClass(LateralPushTest::class.java), LateralPushTest(dvf))
        manager.register(metaForClass(LateralRampLogger::class.java), LateralRampLogger(dvf))
        manager.register(
            metaForClass(ManualFeedforwardTuner::class.java),
            ManualFeedforwardTuner(dvf)
        )
        manager.register(
            metaForClass(MecanumMotorDirectionDebugger::class.java),
            MecanumMotorDirectionDebugger(dvf)
        )
        manager.register(
            metaForClass(DeadWheelDirectionDebugger::class.java),
            DeadWheelDirectionDebugger(dvf)
        )

        manager.register(
            metaForClass(ManualFeedbackTuner::class.java),
            ManualFeedbackTuner::class.java
        )
        manager.register(metaForClass(SplineTest::class.java), SplineTest::class.java)
        manager.register(metaForClass(LocalizationTest::class.java), LocalizationTest::class.java)

        FtcDashboard.getInstance().withConfigRoot { configRoot: CustomVariable ->
            for (c in listOf(
                AngularRampLogger::class.java,
                ForwardRampLogger::class.java,
                LateralRampLogger::class.java,
                ManualFeedforwardTuner::class.java,
                MecanumMotorDirectionDebugger::class.java,
                ManualFeedbackTuner::class.java
            )) {
                configRoot.putVariable(c.simpleName, ReflectionConfig.createVariableFromClass(c))
            }
        }
    }
}
