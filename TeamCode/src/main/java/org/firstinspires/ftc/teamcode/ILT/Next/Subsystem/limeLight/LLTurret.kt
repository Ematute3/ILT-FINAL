package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import kotlin.math.abs


object LLTurret : Subsystem {

    @JvmField var autoAimEnabled = false
    @JvmField var angleToleranceDeg = 1.0
    @JvmField var maxOffsetDeg = 90.0

    var desiredOffsetRad = 0.0
        private set
    var isAligned = false
        private set

    // FIX: Add warning flag
    private var hasWarnedAboutLimelight = false

    override fun periodic() {
        // FIX: Only run if auto-aim is enabled
        if (!autoAimEnabled) {
            isAligned = false
            return
        }

        // FIX: Check if limeLight is ready
        if (!limeLight.isReady()) {
            if (!hasWarnedAboutLimelight) {
                ActiveOpMode.telemetry.addData("LLTurret Warning", "Limelight not initialized")
                hasWarnedAboutLimelight = true
            }
            isAligned = false
            return
        }

        hasWarnedAboutLimelight = false

        // FIX: Check for valid target
        if (!limeLight.hasValidTarget) {
            isAligned = false
            // Don't move turret if no target
            return
        }

        val tx = limeLight.currentTx

        // Check alignment
        isAligned = abs(tx) <= angleToleranceDeg

        // Convert to radians and clamp
        val txRad = Math.toRadians(tx)
        desiredOffsetRad = (-txRad).coerceIn(
            -Math.toRadians(maxOffsetDeg),
            Math.toRadians(maxOffsetDeg)
        )

        // FIX: Set goal and calculate power
        Turret.turretController.goal = KineticState(desiredOffsetRad, 0.0)

        val currentYaw = Turret.getYaw()
        val output = Turret.turretController.calculate(KineticState(currentYaw, 0.0))

        // FIX: Only set power if we're in control
        // IMPORTANT: This assumes Turret.autoAim() is NOT running simultaneously
        Turret.turret.power = output
    }

    // FIX: Don't schedule commands inside InstantCommand
    val enableAutoAim = InstantCommand {
        autoAimEnabled = true
    }

    val disableAutoAim = InstantCommand {
        autoAimEnabled = false
    }

    val toggleAutoAimLL = InstantCommand {
        autoAimEnabled = !autoAimEnabled
    }

    // FIX: Add safety check function
    fun canAutoAim(): Boolean {
        return limeLight.isReady() && limeLight.hasValidTarget
    }

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LL TURRET AUTO-AIM ===")
            appendLine("Enabled: $autoAimEnabled")
            appendLine("Limelight Ready: ${limeLight.isReady()}")
            appendLine("Valid Target: ${limeLight.hasValidTarget}")
            if (limeLight.hasValidTarget) {
                appendLine("TX: ${"%.2f".format(limeLight.currentTx)}°")
                appendLine("Desired Offset: ${"%.1f".format(Math.toDegrees(desiredOffsetRad))}°")
                appendLine("ALIGNED: $isAligned")
            }
        }
    }
}