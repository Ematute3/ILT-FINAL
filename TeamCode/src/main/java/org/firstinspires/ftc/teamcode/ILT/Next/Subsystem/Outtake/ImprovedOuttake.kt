package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.OuttakeMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.seconds

object ImprovedOuttake: SubsystemGroup(FlyWheel, Hood, Turret) {

    // FIX: Use var so it can update
    var goalX = 0.0
        private set

    var goalY = 144.0 - 8.0
        private set

    var mode: OuttakeMode = OuttakeMode.IDLE

    override fun initialize() {
        goalX = if (DriveTrain.alliance == Alliance.RED) {
            144.0 - 6.0
        } else {
            6.0
        }
    }

    // FIX: Make these computed properties that calculate fresh values each time
    val distanceToGoalOdometry: Double
        get() {
            if (!DriveTrain.isPoseValid()) return 0.0
            return sqrt(
                (goalX - DriveTrain.currentX).pow(2) +
                        (goalY - DriveTrain.currentY).pow(2)
            )
        }

    val distanceToGoalLimelight: Double?
        get() = LLAutoVelo.distanceToGoal  // FIX: Keep nullable, don't force unwrap

    val distanceDifference: Double
        get() {
            val llDist = distanceToGoalLimelight
            return if (llDist != null) {
                distanceToGoalOdometry - llDist
            } else {
                0.0
            }
        }

    // FIX: Make these functions that calculate fresh each time
    fun getOdometryAimValues(): DoubleArray? {
        if (!DriveTrain.isPoseValid()) return null
        return Aimbot.getValues(distanceToGoalOdometry)
    }

    fun getLimelightAimValues(): DoubleArray? {
        val llDist = distanceToGoalLimelight ?: return null
        return Aimbot.getValues(llDist)
    }

    // FIX: Safe Limelight-based adjustment
    fun autoHoodFlyLL() {
        val values = getLimelightAimValues()
        if (values != null) {
            Hood.updatePosition(values[0] + 0.06)
            FlyWheel.updatePid(values[1] + 100)
        } else {
            ActiveOpMode.telemetry.addData("LL Auto Aim", "No valid target")
        }
    }

    // FIX: Safe odometry-based adjustment
    fun autoShoot() {
        if (!DriveTrain.isPoseValid()) {
            ActiveOpMode.telemetry.addData("Auto Shoot", "Pose not valid")
            return
        }

        if (DriveTrain.inShootZone()) {
            val values = getOdometryAimValues()
            if (values != null) {
                Hood.updatePosition(values[0] + 0.06)
                FlyWheel.updatePid(values[1] + 100)
                // Note: This doesn't actually shoot, just sets up the parameters
            } else {
                ActiveOpMode.telemetry.addData("Auto Shoot", "No aim values")
            }
        } else {
            ActiveOpMode.telemetry.addData("Auto Shoot", "Not in shoot zone")
        }
    }

    // Manual Aim System
    var targetVelo = 0.0
        private set

    @JvmField var manualAim = 0

    val aimUp = InstantCommand {
        manualAim += 12
        if (manualAim > 146) manualAim = 146
    }

    val aimDown = InstantCommand {
        manualAim -= 12
        if (manualAim < 12) manualAim = 12
    }

    val stopAim = InstantCommand {
        manualAim = 0
    }

    @JvmField var canSpin = true

    fun manualAim() {
        if (mode == OuttakeMode.MANUAL_ADJUST) {
            aimDistance()
            Hood.hS.position = Hood.hP
            Turret.turret.power = Turret.gP
        }
    }

    fun aimDistance() {
        // Snap to nearest valid value
        if (manualAim % 12 != 0) {
            manualAim -= manualAim % 12
        }

        // Clamp to valid range
        if (manualAim > 146) manualAim = 146
        else if (manualAim < 12) manualAim = 12

        // Set velocity based on distance
        if (canSpin) {
            targetVelo = when (manualAim) {
                12 -> 835.0
                24 -> 862.0
                36 -> 844.0
                48 -> 848.0
                60 -> 908.0
                72 -> 1025.0
                84 -> 1165.0
                96 -> 1230.0
                108 -> 1070.0
                120 -> 1112.0
                132 -> 1150.0
                144 -> 1250.0
                else -> 0.0
            }
        }

        // Set hood position based on distance
        Hood.hP = when (manualAim) {
            12 -> 0.81
            24 -> 0.93
            36 -> 0.71
            48 -> 0.6
            60 -> 0.62
            72 -> 0.65
            84 -> 0.7
            96 -> 0.7
            108 -> 0.42
            120 -> 0.43
            132 -> 0.44  // FIX: Was 134
            144 -> 0.45  // FIX: Was 146
            else -> 0.0
        }
    }

    // FIX: Add complete shooting sequence with safety checks
    val fullAutoShootSequence = SequentialGroup(
        // Check we're ready
        InstantCommand {
            if (!DriveTrain.isPoseValid()) {
                ActiveOpMode.telemetry.addData("Shoot Sequence", "Aborting - no pose")
                return@InstantCommand
            }
            if (!DriveTrain.inShootZone()) {
                ActiveOpMode.telemetry.addData("Shoot Sequence", "Not in shoot zone")
                return@InstantCommand
            }
        },

        // Set up hood and flywheel
        InstantCommand { autoShoot() },

        // Spin up flywheel
        FlyWheel.spin,
        Delay(0.5.seconds),

        // Wait for flywheel to reach speed
        InstantCommand {
            var attempts = 0
            while (!FlyWheel.isAtTargetVelocity() && attempts < 100) {
                Thread.sleep(10)
                attempts++
            }
        },

        // Feed the ball
        Intake.feedShooter,
        Delay(0.5.seconds),

        // Stop everything
        FlyWheel.stop,
        Intake.stopIntake
    )

    override fun periodic() {
        // Add telemetry for debugging
        ActiveOpMode.telemetry.run {
            addData("Outtake Mode", mode)
            addData("Goal Position", "(%.1f, %.1f)".format(goalX, goalY))
            addData("Distance (Odom)", "%.1f".format(distanceToGoalOdometry))
            addData("Distance (LL)", distanceToGoalLimelight?.let { "%.1f".format(it) } ?: "N/A")
            addData("In Shoot Zone", DriveTrain.inShootZone())
            addData("Manual Aim Distance", manualAim)
        }
    }
}