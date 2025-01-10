package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.toDegrees
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30.0, 30.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                .build()

        myBot.runAction(
            myBot.drive.actionBuilder(Pose2d(14.0, -62.0, 90.0.toRadians()))

                .splineToLinearHeading(Pose2d(0.0,-34.0,90.0.toRadians()),90.0.toRadians())
                .waitSeconds(0.1)
                .splineToLinearHeading(Pose2d(16.0,-60.0,0.0.toRadians()),0.0.toRadians())
                .splineToLinearHeading(Pose2d(34.0,0.0,0.0.toRadians()),90.0.toRadians())
                .splineToLinearHeading(Pose2d(24.0,0.0,0.0.toRadians()),90.0.toRadians())
                .build()
        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}