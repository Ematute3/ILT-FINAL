package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import kotlin.math.*

// FIX: This subsystem should USE the shared limeLight instance, not create its own
object LLAutoVelo : Subsystem {

    // FIX: Remove duplicate Limelight instance - use limeLight object instead

    // These are all the basic physical configuration values for the Limelight.
    @JvmField var llAngle = 10.0           // Tilt angle of the Limelight in degrees
    @JvmField var llLensHeight = 12.9760551181102// Height of Limelight lens from ground (inches)
    @JvmField var goalHeight = 39.5            // Height of scoring target (inches)

    // Physics calculation constants
    private const val LAUNCH_ANGLE_DEG = 34.36
    // min 33.767
    // max 70 ish
    private const val SHOOTER_HEIGHT_IN = 12.4462122047
    private const val GOAL_HEIGHT_IN = 39.5
    private const val GRAVITY_IN_PER_S2 = 386.0
    private const val SHOOTER_DIAMETER_IN = 6.0
    private const val SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0

    // Motor configuration
    @JvmField var motorTicksPerRev = 28.0

    // Distance tracking variables
    @JvmField var targetDistance = 24.0
    @JvmField var distanceTolerance = 1.5

    var angleToGoalDegrees: Double = 0.0
        private set
    var angleToGoalRadians: Double = 0.0
        private set

    // FIX: Make properly nullable
    var distanceToGoal: Double? = null
        private set

    var isAtTargetDistance: Boolean = false
        private set

    // FIX:7 Get these from shared limeLight object instead of duplicating
    val currentTy: Double
        get() = limeLight.currentTy

    val hasValidTarget: Boolean
        get() = limeLight.hasValidTarget

    // Calculated results
    var calculatedVelocity: Double = 0.0
        private set
    var calculatedRPM: Double = 0.0
        private set

    // FIX: No initialization needed - we use the shared limeLight instance
    override fun initialize() {
        // Check that limeLight is initialized
        if (!limeLight.isReady()) {
            ActiveOpMode.telemetry.addData("LLAutoVelo Warning", "limeLight not initialized")
        }
    }

    override fun periodic() {
        // FIX: Only run if limeLight is ready
        if (!limeLight.isReady()) {
            resetValues()
            return
        }

        updateDistanceCalculation()
        updateVelocityCalculation()
    }

    // FIX: Add helper to reset all values
    private fun resetValues() {
        distanceToGoal = null
        angleToGoalDegrees = 0.0
        angleToGoalRadians = 0.0
        isAtTargetDistance = false
        calculatedVelocity = 0.0
        calculatedRPM = 0.0
    }

    private fun updateDistanceCalculation() {
        // FIX: Use limeLight's data instead of separate instance
        if (limeLight.hasValidTarget) {
            val targetOffsetAngleVertical = limeLight.currentTy

            angleToGoalDegrees = llAngle + targetOffsetAngleVertical
            angleToGoalRadians = angleToGoalDegrees * (PI / 180.0)

            // FIX: Check for valid angle before calculating
            if (abs(angleToGoalRadians) < PI / 2) {  // Prevent tan() from going to infinity
                val tanAngle = tan(angleToGoalRadians)
                if (abs(tanAngle) > 0.001) {  // Prevent division by near-zero
                    distanceToGoal = (goalHeight - llLensHeight) / tanAngle

                    // Check if distance is reasonable (positive and not too far)
                    distanceToGoal?.let { dist ->
                        if (dist > 0 && dist < 200.0) {  // Sanity check
                            isAtTargetDistance = abs(dist - targetDistance) <= distanceTolerance
                        } else {
                            distanceToGoal = null
                            isAtTargetDistance = false
                        }
                    }
                } else {
                    distanceToGoal = null
                    isAtTargetDistance = false
                }
            } else {
                distanceToGoal = null
                isAtTargetDistance = false
            }
        } else {
            resetValues()
        }
    }

    private fun updateVelocityCalculation() {
        val distance = distanceToGoal
        if (distance != null && distance > 0) {
            calculatedVelocity = calculateMotorVelocity(distance)
            calculatedRPM = calculateRPM(distance)
        } else {
            calculatedVelocity = 0.0
            calculatedRPM = 0.0
        }
    }

    fun calculateRPM(distanceToTarget: Double): Double {
        val theta = Math.toRadians(LAUNCH_ANGLE_DEG)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)

        val heightDiff = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN

        val numerator = GRAVITY_IN_PER_S2 * distanceToTarget * distanceToTarget
        val denominator = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - heightDiff)

        // If denominator <= 0, shot path is impossible
        if (denominator <= 0) return 0.0

        val velocityInPerSec = sqrt(numerator / denominator)
        val rpm = 60.0 * velocityInPerSec / (2.0 * PI * SHOOTER_RADIUS_IN)

        return rpm
    }

    fun calculateMotorVelocity(distanceToTarget: Double): Double {
        val rpm = calculateRPM(distanceToTarget)
        if (rpm == 0.0) return 0.0

        val ticksPerSec = (rpm / 60.0) * motorTicksPerRev
        return ticksPerSec
    }

    fun getDistanceDebugInfo(): String {
        return buildString {
            appendLine("=== DISTANCE DEBUG ===")
            appendLine("Limelight Ready: ${limeLight.isReady()}")
            appendLine("Valid Target: $hasValidTarget")
            appendLine("llAngle: $llAngle°")
            appendLine("TY: ${"%.2f".format(currentTy)}°")
            appendLine("Angle to Goal: ${"%.2f".format(angleToGoalDegrees)}°")
            appendLine("Angle (radians): ${"%.4f".format(angleToGoalRadians)}")
            appendLine("Goal Height: $goalHeight\"")
            appendLine("Lens Height: $llLensHeight\"")
            appendLine("Height Diff: ${goalHeight - llLensHeight}\"")
            appendLine("tan(angle): ${"%.4f".format(tan(angleToGoalRadians))}")
            appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"}\"")
            appendLine()
            appendLine("=== VELOCITY CALCULATION ===")
            appendLine("Calculated RPM: ${"%.0f".format(calculatedRPM)}")
            appendLine("Calculated Velocity: ${"%.0f".format(calculatedVelocity)} ticks/sec")
            appendLine("Motor TPR: $motorTicksPerRev")
            appendLine("Launch Angle: $LAUNCH_ANGLE_DEG°")
            appendLine("Shooter Height: ${"%.2f".format(SHOOTER_HEIGHT_IN)}\"")
            appendLine("Target Height: $GOAL_HEIGHT_IN\"")
        }
    }

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT AUTO VELO ===")
            appendLine("Limelight Ready: ${limeLight.isReady()}")
            appendLine("Valid Target: $hasValidTarget")
            if (hasValidTarget) {
                appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} inches")
                appendLine("Required RPM: ${"%.0f".format(calculatedRPM)}")
                appendLine("Required Velocity: ${"%.0f".format(calculatedVelocity)} ticks/sec")
                appendLine("At Target Distance: $isAtTargetDistance")
            } else {
                appendLine("No target detected")
            }
        }
    }
}