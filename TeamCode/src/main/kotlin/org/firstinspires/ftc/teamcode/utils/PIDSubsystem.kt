package org.firstinspires.ftc.teamcode.utils

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDFController

abstract class PIDSubsystem(
    private val controller: PIDFController,
) : SubsystemBase() {
    private var enabled: Boolean = true
    private var setpoint: Double = controller.setPoint
        set(value) {
            controller.setPoint = value
            field = value
        }

    protected abstract fun useOutput(output: Double, setpoint: Double)

    protected abstract fun getMeasurement(): Double

    override fun periodic() {
        if (enabled) {
            useOutput(controller.calculate(getMeasurement()), setpoint)
        }
    }

    fun enable() {
        enabled = true
        controller.reset()
    }

    fun disable() {
        enabled = false
        useOutput(0.0, 0.0)
    }

    fun isEnabled(): Boolean {
        return enabled
    }
}