package org.firstinspires.ftc.teamcode.legacy.roadrunner.utils.roadrunner.util

/**
 * Too many people use sleep(...) where it doesn't need to be.
 * This is a simple util that runs an action until a time is reached and then performs a specified stop action.
 * This is basically the same as a chain of commands with a deadline WaitCommand but without command-based.
 *
 * @author Jackson
 */
//class TimedAction @JvmOverloads constructor(
//    onRun: Runnable,
//    onEnd: Runnable,
//    milliseconds: Double,
//    symmetric: Boolean = false,
//) {
//    private val onRun: Runnable
//    private val onEnd: Runnable
//    private val waitTime: Double
//    private val symmetric: Boolean
//    private val timer: ElapsedTime = ElapsedTime()
//    private var state = State.IDLE
//    /**
//     * Sets up a timed action that runs two actions depending on the time
//     * and is non-blocking.
//     *
//     * @param onRun        the action to run during the time period
//     * @param onEnd        the action to run after the time period is up
//     * @param milliseconds the wait time
//     * @param symmetric    true if the timed action runs symmetric
//     */
//    /**
//     * Sets up an asymmetric timed action.
//     *
//     * @param onRun        the action to run during the time period
//     * @param onEnd        the action to run after the time period is up
//     * @param milliseconds the wait time
//     */
//    init {
//        this.onRun = onRun
//        this.onEnd = onEnd
//        waitTime = milliseconds
//        this.symmetric = symmetric
//    }
//
//    /**
//     * @return true if the timed action is currently running
//     */
//    fun running(): Boolean {
//        return state != State.IDLE
//    }
//
//    /**
//     * Resets the timed action to the RUNNING state
//     */
//    fun reset() {
//        timer.reset()
//        state = State.RUNNING
//    }
//
//    /**
//     * Runs the timed action in a non-blocking FSM
//     */
//    fun run() {
//        when (state) {
//            State.IDLE -> {}
//            State.RUNNING -> if (timer.milliseconds() <= waitTime) {
//                onRun.run()
//            } else {
//                timer.reset()
//                onEnd.run()
//                state = if (symmetric) State.SYMMETRIC else State.IDLE
//            }
//
//            State.SYMMETRIC -> if (timer.milliseconds() <= waitTime) {
//                onEnd.run()
//            } else {
//                timer.reset()
//                state = State.IDLE
//            }
//        }
//    }
//
//    internal enum class State {
//        IDLE, RUNNING, SYMMETRIC
//    }
//}