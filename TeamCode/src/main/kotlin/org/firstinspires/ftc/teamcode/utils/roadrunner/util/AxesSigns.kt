package org.firstinspires.ftc.teamcode.utils.roadrunner.util

/**
 * IMU axes signs in the order XYZ (after remapping).
 */
enum class AxesSigns(val bVal: Int) {
    PPP(0), PPN(1), PNP(2), PNN(3), NPP(4), NPN(5), NNP(6), NNN(7)
}