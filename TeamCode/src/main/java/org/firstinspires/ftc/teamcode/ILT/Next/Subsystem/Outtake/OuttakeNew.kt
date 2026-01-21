package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.OuttakeMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.TurretMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret

object OuttakeNew : Subsystem {

    var adjustMode: OuttakeMode = OuttakeMode.IDLE
        private set

    var turretMode: TurretMode = TurretMode.IDLE
        private set

    // Track if we've enabled LL aim to prevent toggling every frame
    private var llAimWasEnabled = false

    // Track previous mode to detect changes (prevents repeated actions)
    private var previousAdjustMode: OuttakeMode? = null
    private var previousTurretMode: TurretMode? = null

    override fun periodic() {
        // Detect mode changes for one-time actions
        val adjustModeChanged = adjustMode != previousAdjustMode
        val turretModeChanged = turretMode != previousTurretMode

        // Handle outtake adjustment modes
        when (adjustMode) {
            OuttakeMode.IDLE -> {
                // Only reset once when entering IDLE mode
                if (adjustModeChanged) {
                    Hood.hP = 0.0
                    Intake.iP = 0.0
                }
            }

            OuttakeMode.MANUAL_ADJUST -> {
                // Manual control handled by ImprovedOuttake
                ImprovedOuttake.manualAim()
            }

            OuttakeMode.AUTO_ADJUST_MANUAL -> {
                if (DriveTrain.isPoseValid()) {
                    ImprovedOuttake.autoShoot()
                } else {
                    ActiveOpMode.telemetry.addData("Auto Adjust", "Waiting for pose...")
                }
            }

            OuttakeMode.AUTO_ADJUST_LL -> {
                if (limeLight.isReady() && limeLight.hasValidTarget) {
                    ImprovedOuttake.autoHoodFlyLL()
                } else {
                    ActiveOpMode.telemetry.addData("Auto Adjust LL", "No target")
                }
            }
        }

        // Handle turret aiming modes
        when (turretMode) {
            TurretMode.IDLE -> {
                // Stop turret movement
                Turret.turret.power = 0.0
                // Disable LL aim if it was enabled
                if (llAimWasEnabled) {
                    LLTurret.autoAimEnabled = false
                    llAimWasEnabled = false
                }
            }

            TurretMode.LL_AIM -> {
                // Enable LL aim once, not toggle every frame
                if (!llAimWasEnabled) {
                    LLTurret.autoAimEnabled = true
                    llAimWasEnabled = true
                }
                // LLTurret.periodic() handles the actual aiming

                if (!limeLight.isReady()) {
                    ActiveOpMode.telemetry.addData("LL Turret", "Limelight not ready")
                } else if (!limeLight.hasValidTarget) {
                    ActiveOpMode.telemetry.addData("LL Turret", "No target")
                }
            }

            TurretMode.MANUAL_AIM -> {
                // Manual aiming uses gP variable (set by gamepad)
                if (llAimWasEnabled) {
                    LLTurret.autoAimEnabled = false
                    llAimWasEnabled = false
                }
                Turret.turret.power = Turret.gP
            }

            TurretMode.ENCODER_AIM -> {
                if (llAimWasEnabled) {
                    LLTurret.autoAimEnabled = false
                    llAimWasEnabled = false
                }

                if (DriveTrain.isPoseValid()) {
                    Turret.autoAimAbsolute()
                } else {
                    ActiveOpMode.telemetry.addData("Encoder Aim", "Waiting for pose...")
                    Turret.turret.power = 0.0
                }
            }

            TurretMode.PEDRO_AIM -> {
                if (llAimWasEnabled) {
                    LLTurret.autoAimEnabled = false
                    llAimWasEnabled = false
                }

                if (DriveTrain.isPoseValid()) {
                    Turret.autoAim()
                } else {
                    ActiveOpMode.telemetry.addData("Pedro Aim", "Waiting for pose...")
                    Turret.turret.power = 0.0
                }
            }
        }

        // Update previous modes for next cycle
        previousAdjustMode = adjustMode
        previousTurretMode = turretMode

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("=== OUTTAKE STATUS ===", "")
            addData("Adjust Mode", adjustMode)
            addData("Turret Mode", turretMode)
            addData("LL Aim Active", LLTurret.autoAimEnabled)
            addData("Pose Valid", DriveTrain.isPoseValid())
            addData("LL Ready", limeLight.isReady())
        }
    }

    // Safe mode switching commands
    val idleMode = InstantCommand {
        adjustMode = OuttakeMode.IDLE
        turretMode = TurretMode.IDLE
    }

    val manualMode = InstantCommand {
        adjustMode = OuttakeMode.MANUAL_ADJUST
        turretMode = TurretMode.MANUAL_AIM
    }

    val autoModeManual = InstantCommand {
        if (!DriveTrain.isPoseValid()) {
            ActiveOpMode.telemetry.addData("Mode Switch", "Cannot use auto - no pose")
            return@InstantCommand
        }
        adjustMode = OuttakeMode.AUTO_ADJUST_MANUAL
        turretMode = TurretMode.ENCODER_AIM
    }

    val autoModePedro = InstantCommand {
        if (!DriveTrain.isPoseValid()) {
            ActiveOpMode.telemetry.addData("Mode Switch", "Cannot use Pedro aim - no pose")
            return@InstantCommand
        }
        adjustMode = OuttakeMode.AUTO_ADJUST_MANUAL
        turretMode = TurretMode.PEDRO_AIM
    }

    val autoModeLL = InstantCommand {
        if (!limeLight.isReady()) {
            ActiveOpMode.telemetry.addData("Mode Switch", "Cannot use LL - not ready")
            return@InstantCommand
        }
        adjustMode = OuttakeMode.AUTO_ADJUST_LL
        turretMode = TurretMode.LL_AIM
    }

    // Helper functions
    fun setAdjustMode(mode: OuttakeMode) {
        adjustMode = mode
    }

    fun setTurretMode(mode: TurretMode) {
        // Clean up previous mode
        if (turretMode == TurretMode.LL_AIM) {
            LLTurret.autoAimEnabled = false
            llAimWasEnabled = false
        }
        turretMode = mode
    }

    fun setModes(adjust: OuttakeMode, turret: TurretMode) {
        setAdjustMode(adjust)
        setTurretMode(turret)
    }

    fun canUseAutoModes(): Boolean {
        return DriveTrain.isPoseValid()
    }

    fun canUseLLModes(): Boolean {
        return limeLight.isReady() && limeLight.hasValidTarget
    }
}