package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.MotorFeedforward
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
import com.acmerobotics.roadrunner.ftc.PinpointEncoder
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.PinpointDrive


object TuningOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    val DRIVE_CLASS = DriveSubsystem.drive::class.java

    val GROUP = "quickstart"
    val DISABLED = false

    @JvmStatic
    private fun metaForClass(cls: Class<out OpMode>): OpModeMeta {
        return OpModeMeta.Builder()
            .setName(cls.simpleName)
            .setGroup(GROUP)
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .setSource(OpModeMeta.Source.EXTERNAL_LIBRARY)
            .build()
    }

    @OpModeRegistrar
    @JvmStatic
    fun register(manager: OpModeManager) {
        if (DISABLED) return

         val dvf = when (DRIVE_CLASS) {
             PinpointDrive::class.java -> object : DriveViewFactory {
                 override fun make(h: HardwareMap): DriveView {
                     DriveSubsystem.initialize(h)

                     val pd = DriveSubsystem.drive as PinpointDrive

                     val leftEncs: List<Encoder> = ArrayList()
                     val rightEncs: List<Encoder> = ArrayList()
                     val parEncs: MutableList<Encoder> = ArrayList()
                     val perpEncs: MutableList<Encoder> = ArrayList()

                     parEncs.add(PinpointEncoder(pd.pinpoint, false, pd.leftRear))
                     perpEncs.add(PinpointEncoder(pd.pinpoint, true, pd.leftRear))

                     return DriveView(
                         DriveType.MECANUM,
                         MecanumDrive.PARAMS.inPerTick,
                         MecanumDrive.PARAMS.maxWheelVel,
                         MecanumDrive.PARAMS.minProfileAccel,
                         MecanumDrive.PARAMS.maxProfileAccel,
                         h.getAll(LynxModule::class.java),
                         listOf(
                             pd.leftFront,
                             pd.leftRear
                         ),
                         listOf(
                             pd.rightFront,
                             pd.rightRear
                         ),
                         leftEncs,
                         rightEncs,
                         parEncs,
                         perpEncs,
                         pd.lazyImu,
                         pd.voltageSensor
                     ) {
                         MotorFeedforward(
                             MecanumDrive.PARAMS.kS,
                             MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                             MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick
                         )
                     }
                 }
             }

             MecanumDrive::class.java -> object : DriveViewFactory {
                override fun make(h: HardwareMap): DriveView {
                    DriveSubsystem.initialize(h)

                    val md = DriveSubsystem.drive

                    val leftEncs: MutableList<Encoder> = ArrayList()
                    val rightEncs: MutableList<Encoder> = ArrayList()
                    val parEncs: MutableList<Encoder> = ArrayList()
                    val perpEncs: MutableList<Encoder> = ArrayList()

                    when (md.localizer) {
                        is MecanumDrive.DriveLocalizer -> {
                            val dl = md.localizer
                            leftEncs.add(dl.leftFront)
                            leftEncs.add(dl.leftRear)
                            rightEncs.add(dl.rightFront)
                            rightEncs.add(dl.rightRear)
                        }

                        is ThreeDeadWheelLocalizer -> {
                            val dl = md.localizer
                            parEncs.add(dl.left)
                            parEncs.add(dl.right)
                            perpEncs.add(dl.strafe)
                        }

                        else -> {
                            throw RuntimeException("unknown localizer: " + md.localizer.javaClass.name)
                        }
                    }

                    return DriveView(
                        DriveType.MECANUM,
                        MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.maxWheelVel,
                        MecanumDrive.PARAMS.minProfileAccel,
                        MecanumDrive.PARAMS.maxProfileAccel,
                        h.getAll(LynxModule::class.java),
                        listOf(
                            md.leftFront,
                            md.leftRear
                        ),
                        listOf(
                            md.rightFront,
                            md.rightRear
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
