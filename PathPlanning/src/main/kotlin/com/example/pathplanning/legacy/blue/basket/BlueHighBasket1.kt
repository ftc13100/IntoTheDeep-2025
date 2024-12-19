package com.example.pathplanning.legacy.blue.basket

import com.acmerobotics.roadrunner.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object BlueHighBasket1 {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setStartPose(Pose2d(12.0, 62.0, (270.0).toRadians()))
            .setConstraints(30.0, 30.0, Math.toRadians(170.0), Math.toRadians(170.0), 13.94)
            .build()

        myBot.runAction(
            myBot.drive.actionBuilder(
                Pose2d(12.0, 62.0, (270.0).toRadians())
            )
                .turn((90.0).toRadians())
                .lineToX(35.0)
                .waitSeconds(2.0)
                .build()
        )


            meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start()
    }
}