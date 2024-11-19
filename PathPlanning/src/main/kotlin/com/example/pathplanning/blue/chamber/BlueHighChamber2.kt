package com.example.pathplanning.blue.chamber

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.toRadians
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.awt.Image
import java.io.File
import java.io.IOException
import javax.imageio.ImageIO

object BlueHighChamber2 {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(30.0, 30.0, Math.toRadians(170.0), Math.toRadians(170.0), 13.94)
            .followTrajectorySequence { drive ->
                drive.trajectorySequenceBuilder(Pose2d(12.0, 62.0, (270.0).toRadians()))
                    .forward(28.0)
                    .strafeRight(11.0)
                    .waitSeconds(2.0)
                    .build()
            }
        var img: Image? = null
        try {
            img = ImageIO.read(File("/Users/ishaanghaskadbi/Downloads/field.png"))
        } catch (_ : IOException)  {

        }

        if (img != null) {
            meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start()
        }
    }
}