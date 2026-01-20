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

    // Current pose values from Pedro odometry
    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0

    override fun initialize() {
        // Initialize zone checker based on alliance
        zoneChecker = ZoneChecker(16.0,16.0)

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
        // Update current position from Pedro follower (with null safety)
        PedroComponent.follower?.let { follower ->
            currentX = follower.pose.x
            currentY = follower.pose.y
            currentHeading = follower.heading
        }

        // Optional: Add telemetry for debugging
        ActiveOpMode.telemetry.run {
            addData("X", "%.2f".format(currentX))
            addData("Y", "%.2f".format(currentY))
            addData("Heading", "%.2f°".format(Math.toDegrees(currentHeading)))
            addData("In Shoot Zone", inShootZone())
        }
    }


    fun inShootZone(): Boolean {
        return if (::zoneChecker.isInitialized) {
            zoneChecker.inShootZone(currentX, currentY)
        } else {
            false
        }
    }


    fun resetImu() {
        imu.zero()  // or imu.reset() depending on NextFTC API
    }

    fun setAlliance(newAlliance: Alliance) {
        alliance = newAlliance
        zoneChecker = ZoneChecker(16.0,16.0)
    }
}