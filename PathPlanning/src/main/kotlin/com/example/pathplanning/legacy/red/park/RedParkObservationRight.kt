package com.example.pathplanning.legacy.red.park

//object RedParkObservationRight {
//    @JvmStatic
//    fun main(args: Array<String>) {
//        val meepMeep = MeepMeep(800)
//
//        val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//            .setConstraints(30.0, 30.0, Math.toRadians(170.0), Math.toRadians(170.0), 13.94)
//            .followTrajectorySequence { drive ->
//                drive.trajectorySequenceBuilder(Pose2d(12.0, -62.0, (90.0).toRadians()))
//                    .strafeRight(48.0)
//                    .build()
//            }
//        var img: Image? = null
//        try {
//            img = ImageIO.read(File("/Users/ishaanghaskadbi/Downloads/field.png"))
//        } catch (_ : IOException)  {
//
//        }
//
//        if (img != null) {
//            meepMeep.setBackground(img)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start()
//        }
//    }
//}