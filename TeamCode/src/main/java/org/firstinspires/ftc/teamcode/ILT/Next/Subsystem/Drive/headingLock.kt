package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive

import com.pedropathing.control.PIDFController
import com.pedropathing.math.MathFunctions
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake
import java.util.function.Supplier
import kotlin.math.atan2

class HeadingLock @JvmOverloads constructor(
    drivePower: Supplier<Double>,
    strafePower: Supplier<Double>,
    turnPower: Supplier<Double>,
    private val robotCentric: Boolean = false
) : DriverControlledCommand(drivePower, strafePower, turnPower) {

    private var controller: PIDFController? = null
    var headingLock: Boolean = false

    // Allow custom goal override, otherwise use ImprovedOuttake's goal
    private var customGoalX: Double? = null
    private var customGoalY: Double? = null

    override fun start() {
        // Safe initialization with null check
        PedroComponent.follower?.let { follower ->
            controller = PIDFController(follower.constants.coefficientsHeadingPIDF)
            follower.startTeleopDrive()
        }
    }

    override fun calculateAndSetPowers(powers: DoubleArray) {
        val (drive, strafe, turn) = powers
        val follower = PedroComponent.follower

        // If follower not initialized, can't do anything
        if (follower == null) {
            return
        }

        // Update coefficients in case they changed
        controller?.setCoefficients(follower.constants.coefficientsHeadingPIDF)

        if (headingLock && controller != null) {
            try {
                controller!!.updateError(getHeadingError())
                follower.setTeleOpDrive(
                    drive,
                    strafe,
                    controller!!.run(),
                    robotCentric
                )
            } catch (e: Exception) {
                // Fallback to manual control if heading calculation fails
                follower.setTeleOpDrive(drive, strafe, turn, robotCentric)
            }
        } else {
            follower.setTeleOpDrive(drive, strafe, turn, robotCentric)
        }
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) {
            PedroComponent.follower?.breakFollowing()
        }
    }

    /**
     * Calculate heading error to face the goal.
     * Uses custom goal if set, otherwise uses ImprovedOuttake's goal.
     */
    fun getHeadingError(): Double {
        val follower = PedroComponent.follower ?: return 0.0
        val currentPose = follower.pose

        // Use custom goal if set, otherwise use ImprovedOuttake's goal
        val targetX = customGoalX ?: ImprovedOuttake.goalX
        val targetY = customGoalY ?: ImprovedOuttake.goalY

        val headingGoal = atan2(targetY - currentPose.y, targetX - currentPose.x)

        val headingError = MathFunctions.getTurnDirection(
            follower.pose.heading,
            headingGoal
        ) * MathFunctions.getSmallestAngleDifference(
            follower.pose.heading,
            headingGoal
        )

        return headingError
    }

    /**
     * Set a custom goal for heading lock (useful for aiming at specific targets).
     */
    fun setCustomGoal(x: Double, y: Double) {
        customGoalX = x
        customGoalY = y
    }

    /**
     * Clear custom goal and revert to using ImprovedOuttake's goal.
     */
    fun clearCustomGoal() {
        customGoalX = null
        customGoalY = null
    }

    /**
     * Check if heading lock can function (follower initialized).
     */
    fun canHeadingLock(): Boolean {
        return PedroComponent.follower != null && controller != null
    }
}