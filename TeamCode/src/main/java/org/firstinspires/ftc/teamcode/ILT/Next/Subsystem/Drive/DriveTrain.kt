package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance

@Configurable
object DriveTrain: Subsystem {

    // IMU for robot heading (Pedro uses this internally)
    val imu = IMUEx("imu", Direction.RIGHT, Direction.UP)

    // Alliance color - MUST be set during initialization
    @JvmField var alliance = Alliance.RED

    // Zone checker for determining if robot is in shooting zone
    private lateinit var zoneChecker: ZoneChecker

    // Sensitivity multiplier for driver control (optional)
    @JvmField var sensitivity = 1.0

    // FIX: Add initialization flag to prevent usage before ready
    private var poseInitialized = false

    // Current pose values from Pedro odometry
    var currentX = 0.0
        private set
    var currentY = 0.0
        private set
    var currentHeading = 0.0
        private set

    override fun initialize() {
        // Initialize zone checker based on alliance
        zoneChecker = ZoneChecker(alliance, 16.0, 16.0)
        poseInitialized = false

        // Optional: Set alliance from OpMode if needed
        // alliance = (ActiveOpMode.opMode as? YourBaseOpMode)?.alliance ?: Alliance.RED
    }

    override val defaultCommand: Command
        get() = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
            false
        )

    override fun periodic() {
        // FIX: Update current position from Pedro follower with proper error handling
        PedroComponent.follower?.let { follower ->
            try {
                currentX = follower.pose.x
                currentY = follower.pose.y
                currentHeading = follower.heading
                poseInitialized = true
            } catch (e: Exception) {
                ActiveOpMode.telemetry.addData("DriveTrain Error", e.message)
                poseInitialized = false
            }
        } ?: run {
            // Follower not initialized yet
            poseInitialized = false
            ActiveOpMode.telemetry.addData("Warning", "Pedro follower not initialized")
        }

        // Optional: Add telemetry for debugging
        ActiveOpMode.telemetry.run {
            addData("Pose Initialized", poseInitialized)
            addData("X", "%.2f".format(currentX))
            addData("Y", "%.2f".format(currentY))
            addData("Heading", "%.2f°".format(Math.toDegrees(currentHeading)))
            addData("Alliance", alliance)
            addData("In Shoot Zone", inShootZone())
        }
    }

    // FIX: Add safety check for pose validity
    fun isPoseValid(): Boolean {
        return poseInitialized && PedroComponent.follower != null
    }

    fun inShootZone(): Boolean {
        return if (::zoneChecker.isInitialized && poseInitialized) {
            zoneChecker.inShootZone(currentX, currentY)
        } else {
            false
        }
    }

    fun resetImu() {
        imu.zero()
    }

    fun setAlliance(newAlliance: Alliance) {
        alliance = newAlliance
        // FIX: Pass alliance to zone checker
        zoneChecker = ZoneChecker(alliance, 16.0, 16.0)
    }

    // FIX: Helper function to get current pose safely
    fun getPose(): Triple<Double, Double, Double>? {
        return if (isPoseValid()) {
            Triple(currentX, currentY, currentHeading)
        } else {
            null
        }
    }
}