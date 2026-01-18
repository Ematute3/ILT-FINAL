package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import kotlin.math.abs


object LLTurret : Subsystem {


    var autoAimEnabled = false

    var angleToleranceDeg = 1.0        // |tx| <= this => aligned

    var maxOffsetDeg = 90.0           // safety clamp on turret offset




    var desiredOffsetRad = 0.0


    var isAligned = false

    override fun periodic() {


        if (!autoAimEnabled) {

            return
        }


        if (!limeLight.hasValidTarget) {
            isAligned = false

            return
        }


        val tx = limeLight.currentTx



        isAligned = abs(tx) <= angleToleranceDeg


        val txRad = Math.toRadians(tx)


        desiredOffsetRad = (-txRad).coerceIn(
            -Math.toRadians(maxOffsetDeg),
            Math.toRadians(maxOffsetDeg)
        )


        Turret.turretController.goal = KineticState(desiredOffsetRad, 0.0)


        val currentYaw = Turret.getYaw()


        val output = Turret.turretController.calculate(KineticState(currentYaw, 0.0))
        Turret.turret.power = output

    }



    val enableAutoAim = InstantCommand {

        autoAimEnabled = true
    }

    val disableAutoAim = InstantCommand {
        autoAimEnabled = false

    }

    val toggleAutoAimLL = InstantCommand {

        if (autoAimEnabled) {
            disableAutoAim.run()
        } else {
            enableAutoAim.run()
        }
    }

    fun getTelemetryString(): String {
        // This builds a text block we can send to telemetry so we can see what the auto-aim system is doing.
        return buildString {
            appendLine("=== LL TURRET AUTO-AIM ===")
            appendLine("Enabled: $autoAimEnabled")
            appendLine("Valid Target: ${limeLight.hasValidTarget}")
            appendLine("TX: ${"%.2f".format(limeLight.currentTx)}°")
            appendLine("Desired Offset: ${"%.1f".format(Math.toDegrees(desiredOffsetRad))}°")
            appendLine("ALIGNED: $isAligned")
        }
    }
}
