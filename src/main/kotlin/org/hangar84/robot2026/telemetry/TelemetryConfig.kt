package org.hangar84.robot2026.telemetry

import edu.wpi.first.wpilibj.Filesystem
import java.nio.file.Files

data class TelemetryJson(
    val enabled: Map<String, Boolean> = emptyMap(),
    val prefix: Map<String, String> = emptyMap(),
    val rateHz: Map<String, Double> = emptyMap()
)

object TelemetryConfig {
    private const val REL_PATH = "telemetry/telemetry.json"

    // Simple + dependency-free JSON parse (minimal)
    private fun readTextOrNull(): String? {
        val deploy = Filesystem.getDeployDirectory().toPath()
        val file = deploy.resolve(REL_PATH)
        return if (Files.exists(file)) Files.readString(file) else null
    }

    private fun parse(json: String): TelemetryJson {
        fun section(name: String): String? {
            val idx = json.indexOf("\"$name\"")
            if (idx < 0) return null
            val braceStart = json.indexOf("{", idx)
            if (braceStart < 0) return null
            var depth = 0
            for (i in braceStart until json.length) {
                if (json[i] == '{') depth++
                if (json[i] == '}') {
                    depth--
                    if (depth == 0) return json.substring(braceStart, i + 1)
                }
            }
            return null
        }

        fun parseBoolMap(obj: String): Map<String, Boolean> =
            Regex("\"([^\"]+)\"\\s*:\\s*(true|false)")
                .findAll(obj)
                .associate { it.groupValues[1] to (it.groupValues[2] == "true") }

        fun parseStringMap(obj: String): Map<String, String> =
            Regex("\"([^\"]+)\"\\s*:\\s*\"([^\"]*)\"")
                .findAll(obj)
                .associate { it.groupValues[1] to it.groupValues[2] }

        fun parseDoubleMap(obj: String): Map<String, Double> =
            Regex("\"([^\"]+)\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)")
                .findAll(obj)
                .associate { it.groupValues[1] to it.groupValues[2].toDouble() }

        val enabledObj = section("enabled") ?: "{}"
        val prefixObj = section("prefix") ?: "{}"
        val rateObj = section("rateHz") ?: "{}"

        return TelemetryJson(
             parseBoolMap(enabledObj),
            parseStringMap(prefixObj),
            parseDoubleMap(rateObj)
        )
    }

    // Public config (loaded once at startup)
    val cfg: TelemetryJson = run {
        val txt = readTextOrNull()
        if (txt == null) TelemetryJson() else parse(txt)
    }

    fun enabled(group: String, default: Boolean = true): Boolean =
        cfg.enabled[group] ?: default

    fun prefix(group: String, default: String): String =
        cfg.prefix[group] ?: default

    fun rateHz(group: String): Double? =
        cfg.rateHz[group]
}