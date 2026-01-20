package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive

import com.pedropathing.control.PIDFController
import com.pedropathing.math.MathFunctions
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake
import java.util.function.Supplier
import kotlin.math.atan2

class headingLock @JvmOverloads constructor(
    drivePower: Supplier<Double>,
    strafePower: Supplier<Double>,
    turnPower: Supplier<Double>,
    private val robotCentric: Boolean = false
) : DriverControlledCommand(drivePower, strafePower, turnPower) {

    lateinit var controller: PIDFController
    var headingLock: Boolean = false

    // FIX: Remove hardcoded goal, use ImprovedOuttake's goal instead
    private var customGoalX: Double? = null
    private var customGoalY: Double? = null

    override fun start() {
        controller = PIDFController(follower.constants.coefficientsHeadingPIDF)
        follower.startTeleopDrive()
    }

    override fun calculateAndSetPowers(powers: DoubleArray) {
        val (drive, strafe, turn) = powers

        // FIX: Update coefficients in case they changed
        controller.setCoefficients(follower.constants.coefficientsHeadingPIDF)

        if (headingLock) {
            // FIX: Only update error if we have a valid pose
            try {
                controller.updateError(getHeadingError())
                follower.setTeleOpDrive(
                    drive,
                    strafe,
                    controller.run(),
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
        if (interrupted) follower.breakFollowing()
    }

    // FIX: Use ImprovedOuttake's goal or custom goal instead of hardcoded values
    fun getHeadingError(): Double {
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

    // FIX: Allow setting custom goal for heading lock
    fun setCustomGoal(x: Double, y: Double) {
        customGoalX = x
        customGoalY = y
    }

    // Clear custom goal and use ImprovedOuttake's goal
    fun clearCustomGoal() {
        customGoalX = null
        customGoalY = null
    }
}