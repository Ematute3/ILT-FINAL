package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood

@TeleOp(name = "Hood Test", group = "Test")
class HoodTestOp : OpMode() {

    private var lastDpadUpTime = 0L
    private var lastDpadDownTime = 0L
    private val DEBOUNCE_MS = 100L  // Prevent too-fast changes

    override fun init() {
        // FIX: Initialize Hood subsystem
        Hood  // Just accessing it ensures it's loaded

        telemetry.addLine("=== HOOD TEST ===")
        telemetry.addLine()
        telemetry.addLine("CONTROLS:")
        telemetry.addLine("  Left Stick Y - Continuous adjustment")
        telemetry.addLine("  DPad Up - Increment +0.01")
        telemetry.addLine("  DPad Down - Decrement -0.01")
        telemetry.addLine("  A - Set to 0.0 (min)")
        telemetry.addLine("  B - Set to 0.5 (mid)")
        telemetry.addLine("  Y - Set to 1.0 (max)")
        telemetry.addLine("  X - Reset to current position")
        telemetry.addLine()
        telemetry.addLine("Position Range: 0.0 to 1.0")
        telemetry.update()
    }

    override fun start() {
        // Start at mid position
        Hood.updatePosition(0.5)
    }

    override fun loop() {
        // Run Hood's periodic to update servo
        Hood.periodic()

        // === CONTINUOUS CONTROL ===
        // Left stick Y controls hood position continuously
        val stickInput = -gamepad1.left_stick_y.toDouble() * 0.005  // Small multiplier for smooth control
        if (Math.abs(stickInput) > 0.001) {  // Dead zone
            Hood.updatePosition((Hood.hP + stickInput).coerceIn(0.0, 1.0))
        }

        // === INCREMENTAL CONTROL WITH DEBOUNCE ===
        val currentTime = System.currentTimeMillis()

        if (gamepad1.dpad_up && (currentTime - lastDpadUpTime > DEBOUNCE_MS)) {
            Hood.updatePosition((Hood.hP + 0.01).coerceIn(0.0, 1.0))
            lastDpadUpTime = currentTime
        }

        if (gamepad1.dpad_down && (currentTime - lastDpadDownTime > DEBOUNCE_MS)) {
            Hood.updatePosition((Hood.hP - 0.01).coerceIn(0.0, 1.0))
            lastDpadDownTime = currentTime
        }

        // === PRESET POSITIONS ===
        if (gamepad1.a) {
            Hood.updatePosition(0.0)  // Min position
        }
        if (gamepad1.b) {
            Hood.updatePosition(0.5)  // Mid position
        }
        if (gamepad1.y) {
            Hood.updatePosition(1.0)  // Max position
        }
        if (gamepad1.x) {
            // Reset to current servo position (useful for calibration)
            Hood.updatePosition(Hood.hS.position)
        }

        // === TELEMETRY ===
        telemetry.addLine("=== HOOD STATUS ===")
        telemetry.addData("Target Position (hP)", "%.3f".format(Hood.hP))
        telemetry.addData("Actual Servo Pos", "%.3f".format(Hood.hS.position))
        telemetry.addData("Position %", "%.1f%%".format(Hood.hP * 100))
        telemetry.addLine()

        telemetry.addLine("=== CONTROLS ===")
        telemetry.addData("Left Stick Y", "%.3f".format(gamepad1.left_stick_y))
        telemetry.addLine()

        telemetry.addLine("=== PRESETS ===")
        telemetry.addData("A (Min)", "0.000")
        telemetry.addData("B (Mid)", "0.500")
        telemetry.addData("Y (Max)", "1.000")
        telemetry.addLine()

        // FIX: Show calibration values from manual aim
        telemetry.addLine("=== MANUAL AIM POSITIONS ===")
        telemetry.addData("12\" (Close)", "0.810")
        telemetry.addData("24\"", "0.930")
        telemetry.addData("36\"", "0.710")
        telemetry.addData("48\"", "0.600")
        telemetry.addData("60\"", "0.620")
        telemetry.addData("72\"", "0.650")
        telemetry.addData("84\"", "0.700")
        telemetry.addData("96\"", "0.700")
        telemetry.addData("108\"", "0.420")
        telemetry.addData("120\"", "0.430")
        telemetry.addData("132\"", "0.440")
        telemetry.addData("144\" (Far)", "0.450")

        telemetry.update()
    }

    override fun stop() {
        // Return to safe position when stopping
        Hood.updatePosition(0.5)
        Hood.periodic()
    }
}