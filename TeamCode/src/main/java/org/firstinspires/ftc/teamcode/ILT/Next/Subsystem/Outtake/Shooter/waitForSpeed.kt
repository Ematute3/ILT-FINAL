package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.commands.Command
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel

/**
 * Non-blocking command that waits for the flywheel to reach target velocity.
 * Finishes when flywheel is at speed OR timeout is exceeded.
 */
class WaitForFlywheelSpeed(
    private val timeoutMs: Long = 2000L  // 2 second default timeout
) : Command() {

    private var startTime = 0L

    // NextFTC uses isDone property instead of isFinished() method
    override val isDone: Boolean
        get() {
            val elapsed = System.currentTimeMillis() - startTime
            return FlyWheel.isAtTargetVelocity() || elapsed > timeoutMs
        }

    override fun start() {
        startTime = System.currentTimeMillis()
    }

    override fun update() {
        // Nothing to do each cycle - just waiting
        ActiveOpMode.telemetry.addData("Flywheel Wait", "Waiting for speed...")
        ActiveOpMode.telemetry.addData("Current Velocity", "%.0f".format(FlyWheel.f1.velocity))
        ActiveOpMode.telemetry.addData("Target Velocity", "%.0f".format(FlyWheel.targetVelocity))
        ActiveOpMode.telemetry.addData("At Speed", FlyWheel.isAtTargetVelocity())
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) {
            ActiveOpMode.telemetry.addData("Flywheel Wait", "Interrupted")
        } else {
            ActiveOpMode.telemetry.addData("Flywheel Wait",
                if (FlyWheel.isAtTargetVelocity()) "At speed!" else "Timed out"
            )
        }
    }
}