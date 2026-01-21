package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(name = "Turret Test", group = "Test")
class TurretTestOpMode : NextFTCOpMode() {

    // Current commanded yaw (radians)
    private var targetYaw = 0.0

    // Scale from stick input to radians per loop
    private val stickScale = 0.02

    // Control mode
    private enum class ControlMode {
        MANUAL,
        ABSOLUTE,
        PRESET
    }

    private var mode = ControlMode.MANUAL

    init {
        addComponents(
            SubsystemComponent(
                DriveTrain,
                Turret
            ),
            BindingsComponent,
            BulkReadComponent
        )
    }

    override fun onInit() {
        DriveTrain.setAlliance(Alliance.RED)
        DriveTrain.initialize()
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

    override fun onStartButtonPressed() {
        // Mode switching (using proper bindings - no Thread.sleep!)
        val toggleMode = Gamepads.gamepad1.y
        toggleMode.whenBecomesTrue {
            mode = when (mode) {
                ControlMode.MANUAL -> ControlMode.ABSOLUTE
                ControlMode.ABSOLUTE -> ControlMode.PRESET
                ControlMode.PRESET -> ControlMode.MANUAL
            }
        }

        // Preset positions
        val centerPreset = Gamepads.gamepad1.a
        centerPreset.whenBecomesTrue { targetYaw = 0.0 }

        val rightPreset = Gamepads.gamepad1.b
        rightPreset.whenBecomesTrue { targetYaw = Math.toRadians(45.0) }

        val leftPreset = Gamepads.gamepad1.x
        leftPreset.whenBecomesTrue { targetYaw = Math.toRadians(-45.0) }

        // Encoder controls
        val zeroEncoder = Gamepads.gamepad1.dpadUp
        zeroEncoder.whenBecomesTrue { Turret.zeroMotor.schedule() }

        val calibrateAbsolute = Gamepads.gamepad1.dpadDown
        calibrateAbsolute.whenBecomesTrue { Turret.calibrateEncoderCommand.schedule() }
    }

    override fun onUpdate() {
        // Run turret's periodic
        Turret.periodic()

        // Handle control based on mode
        when (mode) {
            ControlMode.MANUAL -> handleManualControl()
            ControlMode.ABSOLUTE -> handleAbsoluteControl()
            ControlMode.PRESET -> handlePresetControl()
        }

        // Telemetry
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
        telemetry.addLine()

        telemetry.addLine("=== CONTROLS ===")
        telemetry.addData("Left Stick X", "%.3f".format(Gamepads.gamepad1.leftStickX.state))

        telemetry.update()
    }

    private fun handleManualControl() {
        val stickX = Gamepads.gamepad1.leftStickX.state

        // Apply dead zone
        if (abs(stickX) > 0.05) {
            targetYaw -= stickX * stickScale
        }

        // Clamp to ±90 degrees
        val maxYaw = PI / 2
        targetYaw = targetYaw.coerceIn(-maxYaw, maxYaw)

        // Send command to turret
        Turret.goToYaw(targetYaw)
    }

    private fun handleAbsoluteControl() {
        val stickX = Gamepads.gamepad1.leftStickX.state

        if (abs(stickX) > 0.05) {
            targetYaw -= stickX * stickScale
        }

        val maxYaw = PI / 2
        targetYaw = targetYaw.coerceIn(-maxYaw, maxYaw)

        // Use absolute encoder version
        Turret.goToYawAbsolute(targetYaw)
    }

    private fun handlePresetControl() {
        // Presets are handled by button bindings in onStartButtonPressed
        Turret.goToYaw(targetYaw)
    }

    override fun onStop() {
        // Stop turret when OpMode ends
        Turret.turret.power = 0.0
    }
}