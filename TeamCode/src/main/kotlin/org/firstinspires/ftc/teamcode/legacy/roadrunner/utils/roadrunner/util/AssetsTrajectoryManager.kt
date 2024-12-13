package org.firstinspires.ftc.teamcode.legacy.roadrunner.utils.roadrunner.util

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
//object AssetsTrajectoryManager {
////    /**
////     * Loads the group config.
////     */
//    fun loadGroupConfig(): TrajectoryGroupConfig? {
//        return try {
//            val inputStream = AppUtil.getDefContext().assets.open(
//                "trajectory/$GROUP_FILENAME"
//            )
//            loadGroupConfig(inputStream)
//        } catch (e: IOException) {
//            null
//        }
//    }
//
//    /**
//     * Loads a trajectory config with the given name.
//     */
//    fun loadConfig(name: String): TrajectoryConfig? {
//        return try {
//            val inputStream = AppUtil.getDefContext().assets.open(
//                "trajectory/$name.yaml"
//            )
//            loadConfig(inputStream)
//        } catch (e: IOException) {
//            null
//        }
//    }
//
//    /**
//     * Loads a trajectory builder with the given name.
//     */
//    fun loadBuilder(name: String): TrajectoryBuilder? {
//        val groupConfig = loadGroupConfig()
//        val config = loadConfig(name)
//        return if (groupConfig == null || config == null) {
//            null
//        } else config.toTrajectoryBuilder(
//            groupConfig
//        )
//    }
//
//    /**
//     * Loads a trajectory with the given name.
//     */
//    fun load(name: String): Trajectory? {
//        val builder = loadBuilder(name) ?: return null
//        return builder.build()
//    }
//}