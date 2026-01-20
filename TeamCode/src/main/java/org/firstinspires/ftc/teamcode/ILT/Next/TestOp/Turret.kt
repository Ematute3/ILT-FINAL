package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import kotlin.math.PI

@TeleOp(name = "Turret Test", group = "Test")
class TurretTestOpMode : OpMode() {

    // Current commanded yaw (radians)
    private var targetYaw = 0.0

    // Scale from stick input to radians per loop
    private val stickScale = 0.02   // Smaller = slower, more precise

    // FIX: Track control mode
    private enum class ControlMode {
        MANUAL,      // Stick control
        ABSOLUTE,    // Using absolute encoder
        PRESET       // Preset positions
    }

    private var mode = ControlMode.MANUAL

    override fun init() {
        // FIX: Initialize DriveTrain for auto-aim testing
        DriveTrain.setAlliance(Alliance.RED)
        DriveTrain.initialize()

        // FIX: Initialize Turret
        Turret.initialize()

        telemetry.addLine("=== TURRET TEST ===")
        telemetry.addLine()
        telemetry.addLine("CONTROLS:")
        telemetry.addLine("  Left Stick X - Manual rotation")
        telemetry.addLine("  A - Center (0°)")
        telemetry.addLine("  B - Right 45°")
        telemetry.addLine("  X - Left 45°")
        telemetry.addLine("  Y - Toggle control mode")
        telemetry.addLine("  DPad Up - Zero encoder")
        telemetry.addLine("  DPad Down - Calibrate absolute encoder")
        telemetry.addLine()
        telemetry.addLine("RANGE: ±90°")
        telemetry.update()
    }

    override fun start() {
        targetYaw = 0.0
        mode = ControlMode.MANUAL
    }

    override fun loop() {
        // Run turret's periodic
        Turret.periodic()

        when (mode) {
            ControlMode.MANUAL -> handleManualControl()
            ControlMode.ABSOLUTE -> handleAbsoluteControl()
            ControlMode.PRESET -> handlePresetControl()
        }

        // === BUTTON CONTROLS ===

        // Mode switching
        if (gamepad1.y) {
            mode = when (mode) {
                ControlMode.MANUAL -> ControlMode.ABSOLUTE
                ControlMode.ABSOLUTE -> ControlMode.PRESET
                ControlMode.PRESET -> ControlMode.MANUAL
            }
            Thread.sleep(200)  // Debounce
        }

        // Preset positions (work in all modes)
        if (gamepad1.a) {
            targetYaw = 0.0  // Center
        }
        if (gamepad1.b) {
            targetYaw = Math.toRadians(45.0)  // Right 45°
        }
        if (gamepad1.x) {
            targetYaw = Math.toRadians(-45.0)  // Left 45°
        }

        // Encoder controls
        if (gamepad1.dpad_up) {
            Turret.zeroMotor
            Thread.sleep(200)
        }
        if (gamepad1.dpad_down) {
            Turret.calibrateAbsoluteEncoder()
            Thread.sleep(200)
        }

        // === TELEMETRY ===
        telemetry.addLine("=== TURRET STATUS ===")
        telemetry.addData("Control Mode", mode)
        telemetry.addLine()

        telemetry.addLine("=== POSITION ===")
        telemetry.addData("Target Yaw", "%.2f°".format(Math.toDegrees(targetYaw)))
        telemetry.addData("Current Yaw", "%.2f°".format(Math.toDegrees(Turret.getYaw())))
        telemetry.addData("Error", "%.2f°".format(
            Math.toDegrees(targetYaw - Turret.getYaw())
        ))
        telemetry.addLine()

        telemetry.addLine("=== MOTOR ===")
        telemetry.addData("Power", "%.3f".format(Turret.turret.power))
        telemetry.addData("Velocity", "%.1f".format(Turret.turret.velocity))
        telemetry.addData("Position", Turret.turret.currentPosition)
        telemetry.addLine()

        telemetry.addLine("=== ENCODERS ===")
        telemetry.addData("Relative Encoder", Turret.turret.currentPosition)
        // FIX: Check if absolute encoder is available
        if (Turret.getAbsolutePositionRatio() > 0.0) {
            telemetry.addData("Absolute Ratio", "%.3f".format(Turret.getAbsolutePositionRatio()))
            telemetry.addData("Absolute Degrees", "%.1f°".format(Turret.getAbsolutePositionDegrees()))
            telemetry.addData("Absolute Yaw", "%.2f°".format(Math.toDegrees(Turret.getAbsoluteYaw())))
        } else {
            telemetry.addData("Absolute Encoder", "Not initialized")
        }
        telemetry.addLine()

        telemetry.addLine("=== CONTROLLER ===")
        telemetry.addData("Goal Position", "%.2f°".format(
            Math.toDegrees(Turret.turretController.goal.position)
        ))
        telemetry.addData("Goal Velocity", "%.2f".format(
            Turret.turretController.goal.velocity
        ))
        telemetry.addLine()

        telemetry.addLine("=== CONTROLS ===")
        telemetry.addData("Left Stick X", "%.3f".format(gamepad1.left_stick_x))

        telemetry.update()
    }

    private fun handleManualControl() {
        // Left stick X: negative -> move turret right, positive -> move turret left
        val stickX = gamepad1.left_stick_x.toDouble()

        // Apply dead zone
        if (Math.abs(stickX) > 0.05) {
            targetYaw -= stickX * stickScale
        }

        // Clamp to ±90 degrees
        val maxYaw = PI / 2
        targetYaw = targetYaw.coerceIn(-maxYaw, maxYaw)

        // Send command to turret
        Turret.goToYaw(targetYaw)
    }

    private fun handleAbsoluteControl() {
        // Same as manual but uses absolute encoder
        val stickX = gamepad1.left_stick_x.toDouble()

        if (Math.abs(stickX) > 0.05) {
            targetYaw -= stickX * stickScale
        }

        val maxYaw = PI / 2
        targetYaw = targetYaw.coerceIn(-maxYaw, maxYaw)

        // FIX: Use absolute encoder version
        Turret.goToYawAbsolute(targetYaw)
    }

    private fun handlePresetControl() {
        // Buttons set presets, turret goes there
        // (Presets are handled in main loop)
        Turret.goToYaw(targetYaw)
    }

    override fun stop() {
        // Stop turret when OpMode ends
        Turret.turret.power = 0.0
    }
}