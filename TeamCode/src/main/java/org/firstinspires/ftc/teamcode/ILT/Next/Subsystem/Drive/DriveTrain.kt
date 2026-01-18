package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.ZoneChecker

@Configurable
object DriveTrain: Subsystem {

    val fL = MotorEx("fl")
    val fR = MotorEx("fr")
    val bL = MotorEx("bl")
    val bR = MotorEx("br")
    val imu = IMUEx("imu", Direction.RIGHT, Direction.UP)
// need to chagne for the new robot

    @JvmField var alliance = Alliance.RED
    private lateinit var zoneChecker: ZoneChecker



    @JvmField var sensitivity = 1.0


    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0
// change these values.


    override val defaultCommand: Command
        get() = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
            false
        )

    override fun periodic() {

        currentX = PedroComponent.Companion.follower.pose.x
        currentY = PedroComponent.Companion.follower.pose.y
        currentHeading = PedroComponent.Companion.follower.heading
    }
    fun inShootZone(): Boolean {
        return if (::zoneChecker.isInitialized) {
            zoneChecker.inShootZone(currentX, currentY)
        } else {
            false
        }
    }





}