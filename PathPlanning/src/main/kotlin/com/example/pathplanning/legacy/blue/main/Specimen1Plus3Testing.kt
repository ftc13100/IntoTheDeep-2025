package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object Specimen1Plus3Testing {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.0, 50.0, Math.toRadians(180.0), Math.toRadians(180.0), 4669.698132939092 * 0.002953423066)
                .build()

        myBot.runAction(
            myBot.drive.actionBuilder(Pose2d(-10.0, 62.0, 90.0.toRadians()))
                .strafeToConstantHeading(Vector2d(2.0, 31.5))
                .setTangent(90.0.toRadians())
                .splineToLinearHeading(Pose2d(-36.0, 35.0, Math.toRadians(90.0)), -90.0.toRadians())
                .setTangent(-90.0.toRadians())
                .splineToLinearHeading(Pose2d(-44.0, 5.0, 90.0.toRadians()), 120.0.toRadians())
                .strafeTo(Vector2d(-44.0, 56.0))
                .setTangent(-90.0.toRadians())
                .splineToLinearHeading( Pose2d(-56.0, 5.0, 90.0.toRadians()), 130.0.toRadians())
                .strafeTo(Vector2d(-56.0, 56.0))
                .setTangent(-90.0.toRadians())
                .splineToLinearHeading(Pose2d(-34.0, 48.0, 180.0.toRadians()), Math.toRadians(90.0))
                .setTangent(90.0.toRadians())
                .strafeTo(Vector2d(-34.0, 59.0))
                .splineToLinearHeading(Pose2d(0.0, 34.0, 90.0.toRadians()), -90.0.toRadians())
                .setTangent(0.0.toRadians())
                .splineToSplineHeading(Pose2d(-34.0, 59.0, 180.0.toRadians()), 0.0)
                .setTangent(180.0.toRadians())
//                .splineToLinearHeading(Pose2d(0.0, 34.0, 90.0.toRadians()), -90.0.toRadians())
//                .setTangent(0.0.toRadians())
//                .splineToSplineHeading(Pose2d(-36.0, 59.0, 180.0.toRadians()), 0.0)
//                .setTangent(180.0.toRadians())
//                .splineToLinearHeading(Pose2d(0.0, 34.0, 90.0.toRadians()), -90.0.toRadians())
                .build()

        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}