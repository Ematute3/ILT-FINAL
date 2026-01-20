package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.OuttakeMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.ShootMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.TurretMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.iP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood.hP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood.hS
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.gP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.getYaw
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.turretController
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret
import kotlin.math.pow
import kotlin.math.sqrt

object OuttakeNew: Subsystem {
    var adjustMode: OuttakeMode = OuttakeMode.IDLE
    var turretMode: TurretMode = TurretMode.IDLE
    //var shootMode: ShootMode = ShootMode.IDLE

    override fun periodic() {
        // Handle outtake adjustment modes
        when(adjustMode) {
            OuttakeMode.IDLE -> {
                // Stop all adjustment
                Hood.stopHood
                Intake.stopIntake

            }
            OuttakeMode.MANUAL_ADJUST -> {
                // Manual control is handled elsewhere (gamepad inputs)
                // Hood uses hP variable, FlyWheel uses manual commands
                ImprovedOuttake.manualAim()

            }
            OuttakeMode.AUTO_ADJUST_MANUAL -> {
                // Auto-adjust based on distance calculation
                ImprovedOuttake.autoShoot()

            }
            OuttakeMode.AUTO_ADJUST_LL -> {
                // Use Limelight for auto-adjustment
                ImprovedOuttake.autoHoodFlyLL()
            }
        }

        // Handle turret aiming modes
        when(turretMode) {
            TurretMode.IDLE -> {
                // Stop turret movement
                Turret.turret.power = 0.0
            }
            TurretMode.LL_AIM -> {
                // Use Limelight for aiming

                Turret.autoAimLL()
            }
            TurretMode.MANUAL_AIM -> {
                // Manual aiming uses gP variable (set by gamepad)
                Turret.turret.power = gP
            }
            TurretMode.ENCODER_AIM -> {
                // Use encoder-based auto-aim
                Turret.autoAimAbsolute()
            }
            TurretMode.PEDRO_AIM ->{
                Turret.autoAim()
            }
      }
        ActiveOpMode.telemetry.run {
            addData("Adjust Mode", adjustMode)
            addData("Turret Mode", turretMode)
        }

    }

    // Quick mode switching commands
    val idleMode = InstantCommand {
        adjustMode = OuttakeMode.IDLE
        turretMode = TurretMode.IDLE
    }

    val manualMode = InstantCommand {
        adjustMode = OuttakeMode.MANUAL_ADJUST
        turretMode = TurretMode.MANUAL_AIM
    }

    val autoModeManual = InstantCommand {
        adjustMode = OuttakeMode.AUTO_ADJUST_MANUAL
        turretMode = TurretMode.ENCODER_AIM
    }

    val autoModeLL = InstantCommand {
        adjustMode = OuttakeMode.AUTO_ADJUST_LL
        turretMode = TurretMode.LL_AIM
    }
}