package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object Sample1Plus3Testing {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.0, 50.0, Math.toRadians(180.0), Math.toRadians(180.0), 4669.698132939092 * 0.002953423066)
                .build()

        myBot.runAction(
            myBot.drive.actionBuilder(Pose2d(38.0, 62.0, -90.0.toRadians()))
                .setTangent(-45.0)
                .splineToLinearHeading(Pose2d(56.0, 56.0, Math.toRadians(225.0)), Math.toRadians(45.0))
//                .turn(Math.toRadians(30.0))
//                .turn(Math.toRadians(-30.0))
//                .turn(Math.toRadians(55.0))
//                .turn(Math.toRadians(-55.0))
//                .turn(Math.toRadians(75.0))
//                .turn(Math.toRadians(-75.0))
                .setTangent(Math.toRadians(-90.0))
                .splineToLinearHeading(Pose2d(25.0, 10.0, 0.0), Math.toRadians(120.0))
                .build()

        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}