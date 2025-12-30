package org.hangar84.robot2026.telemetry

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

interface TelemetrySink {
    fun putNumber(key: String, value: Double)
    fun putBoolean(key: String, value: Boolean)
    fun putString(key: String, value: String)
}

/** Default sink: SmartDashboard (NetworkTables). */
object SmartDashboardSink : TelemetrySink {
    override fun putNumber(key: String, value: Double) {
        SmartDashboard.putNumber(key, value)
    }
    override fun putBoolean(key: String, value: Boolean) {
        SmartDashboard.putBoolean(key, value)
    }
    override fun putString(key: String, value: String) {
        SmartDashboard.putString(key, value)
    }
}

object TelemetrySinks {
    private val sinks: MutableList<TelemetrySink> = mutableListOf(SmartDashboardSink)

    fun register(sink: TelemetrySink) { sinks += sink }

    fun publishNumber(key: String, value: Double) { sinks.forEach { it.putNumber(key, value) } }
    fun publishBoolean(key: String, value: Boolean) { sinks.forEach { it.putBoolean(key, value) } }
    fun publishString(key: String, value: String) { sinks.forEach { it.putString(key, value) } }
}