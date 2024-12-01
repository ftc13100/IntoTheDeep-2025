package com.example.pathplanning.blue.main

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.awt.Image
import java.io.File
import java.io.IOException
import javax.imageio.ImageIO

object BlueAuto {
    @JvmStatic

    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30.0, 30.0, Math.toRadians(170.0), Math.toRadians(170.0), 13.94)
                .followTrajectorySequence { drive ->
                    drive.trajectorySequenceBuilder(Pose2d(
                        -12.0, 62.0, Math.toRadians(270.0)
                    ))
                        .addTemporalMarker(0.1,) {
                            //add the command for the arm to go up
                            //add command for belt servo to go backwards
                            //add the command to add the slides go up
                        }
                        .splineToLinearHeading(Pose2d(53.0, 55.0, Math.toRadians(225.0)), Math.toRadians(0.0))
                        .addDisplacementMarker {
                            //add command for rollers servo to outtake
                        }
//                        .turn(Math.toRadians(30.0)) //
//                        .addTemporalMarker(6.0) {
//                            //add command for arm to go down
//                            //add command for slides to go out
//                            //add command to belt servo to go at [x] angle
//                        }
//                        .addDisplacementMarker {
//                            //add command to intake
//                        }
//                        .turn(Math.toRadians(-30.0)) //
//                        .addTemporalMarker() {
//                            //add command to bring slides in
//                            //add command to bring arm up
//                            //add command to put slides out
//                            //add command to turn belt servo
//                        }
//                        .addDisplacementMarker {
//                            //add command to outtake
//                        }
//                        .turn(Math.toRadians(55.0)) //
//                        .addTemporalMarker() {
//                            //add command to bring slides in
//                            //add command to bring arm down
//                            //add command to put slides out
//                            //add command to turn belt servo
//                        }
//                        .addDisplacementMarker {
//                            //add command to intake
//                        }
//                        .turn(Math.toRadians(-55.0)) //
//                        .addTemporalMarker() {
//                            //add command to bring slides in
//                            //add command to bring arm up
//                            //add command to put slides out
//                            //add command to turn belt servo
//                        }
//                        .addDisplacementMarker {
//                            //add command to outtake
//                        }
//                        .turn(Math.toRadians(80.0)) //
//                        .addTemporalMarker() {
//                            //add command to bring slides in
//                            //add command to bring arm down
//                            //add command to put slides out
//                            //add command to turn belt servo
//                        }
//                        .addDisplacementMarker {
//                            //add command to intake
//                        }
//                        .turn(Math.toRadians(-80.0)) //
//                        .addTemporalMarker() {
//                            //add command to bring slides in
//                            //add command to bring arm up
//                            //add command to put slides out
//                            //add command to turn belt servo
//                        }
//                        .addDisplacementMarker {
//                            //add command to outtake
//                        }
                        .lineToSplineHeading(Pose2d(36.0, 8.0, Math.toRadians(180.0)))




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