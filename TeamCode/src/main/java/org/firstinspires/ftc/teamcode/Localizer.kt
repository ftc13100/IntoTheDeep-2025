package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual

interface Localizer {
    fun update(): Twist2dDual<Time>
}
