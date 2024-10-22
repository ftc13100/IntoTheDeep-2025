package org.firstinspires.ftc.teamcode.utils.roadrunner.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.system.Misc

/**
 * Collection of utilites for interacting with Lynx modules.
 */
object LynxModuleUtil {
    private val MIN_VERSION = LynxFirmwareVersion(1, 8, 2)

    /**
     * Retrieve and parse Lynx module firmware version.
     *
     * @param module Lynx module
     * @return parsed firmware version
     */
    fun getFirmwareVersion(module: LynxModule): LynxFirmwareVersion? {
        val versionString = module.nullableFirmwareVersionString ?: return null
        val parts =
            versionString.split("[ :,]+".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
        return try {
            // note: for now, we ignore the hardware entry
            LynxFirmwareVersion(parts[3].toInt(), parts[5].toInt(), parts[7].toInt())
        } catch (e: NumberFormatException) {
            null
        }
    }

    /**
     * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
     *
     * @param hardwareMap hardware map containing Lynx modules
     */
    fun ensureMinimumFirmwareVersion(hardwareMap: HardwareMap) {
        val outdatedModules: MutableMap<String, LynxFirmwareVersion?> = HashMap()
        for (module in hardwareMap.getAll<LynxModule>(LynxModule::class.java)) {
            val version = getFirmwareVersion(module)
            if (version == null || version < MIN_VERSION) {
                for (name in hardwareMap.getNamesOf(module)) {
                    outdatedModules[name] = version
                }
            }
        }
        if (outdatedModules.isNotEmpty()) {
            val msgBuilder = StringBuilder()
            msgBuilder.append("One or more of the attached Lynx modules has outdated firmware\n")
            msgBuilder.append(
                Misc.formatInvariant(
                    "Mandatory minimum firmware version for Road Runner: %s\n",
                    MIN_VERSION.toString()
                )
            )
            for ((key, value) in outdatedModules) {
                msgBuilder.append(
                    Misc.formatInvariant(
                        "\t%s: %s\n", key,
                        value?.toString() ?: "Unknown"
                    )
                )
            }
            throw LynxFirmwareVersionException(msgBuilder.toString())
        }
    }

    /**
     * Parsed representation of a Lynx module firmware version.
     */
    class LynxFirmwareVersion(val major: Int, val minor: Int, val eng: Int) :
        Comparable<LynxFirmwareVersion> {
        override fun equals(other: Any?): Boolean {
            return if (other is LynxFirmwareVersion) {
                major == other.major && minor == other.minor && eng == other.eng
            } else {
                false
            }
        }

        override fun compareTo(other: LynxFirmwareVersion): Int {
            val majorComp = major.compareTo(other.major)
            return if (majorComp == 0) {
                val minorComp = minor.compareTo(other.minor)
                if (minorComp == 0) {
                    eng.compareTo(other.eng)
                } else {
                    minorComp
                }
            } else {
                majorComp
            }
        }

        override fun toString(): String {
            return Misc.formatInvariant("%d.%d.%d", major, minor, eng)
        }
    }

    /**
     * Exception indicating an outdated Lynx firmware version.
     */
    class LynxFirmwareVersionException(detailMessage: String?) : RuntimeException(detailMessage)
}