package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake.goalY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake.goalX
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret
import kotlin.math.PI
import kotlin.math.atan2


object Turret: Subsystem {

    val turret = MotorEx("turret")

    // FIX: Make nullable to handle initialization safely
    private var turretEncoder: AnalogInput? = null
    var encoderOffset = 0.0
    var gP = 0.0

    private val gearRatio = 3.62068965517 // 105/29

    @JvmField
    var autoTurret = true

    @JvmField
    var turretPID = PIDCoefficients(0.011, 0.0, 0.2)

    // FIX: Add flag to track if we've warned about missing pose
    private var hasWarnedAboutPose = false

    override fun initialize() {
        try {
            // FIX: Safe initialization with try-catch
            turretEncoder = ActiveOpMode.hardwareMap.get(AnalogInput::class.java, "encoder")
            encoderOffset = 0.0
            ActiveOpMode.telemetry.addData("Turret Encoder", "Initialized Successfully")
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("Turret Encoder Error", e.message)
            turretEncoder = null
        }
    }

    var turretController = controlSystem {
        posPid(turretPID)
    }

    private val ppr = 537.7
    private val rpt = 2 * PI / (ppr * gearRatio)

    override fun periodic() {
        // FIX: Only display if encoder is initialized
        ActiveOpMode.telemetry.run {
            addData("goal", turretController.goal.position)
            addData("turret Pos", getYaw())
            addData("encoder initialized", turretEncoder != null)
            addData("pose valid", DriveTrain.isPoseValid())
            if (turretEncoder != null) {
                addData("absolute yaw", getAbsoluteYaw())
            }
        }
    }

    fun autoAimLL() {
        LLTurret.toggleAutoAimLL()
    }

    val zeroMotor = InstantCommand {
        turret.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turret.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    val spinGearLeft = InstantCommand { gP = 0.6 }
    val spinGearRight = InstantCommand { gP = -0.6 }
    val gearAlittleLeft = InstantCommand { gP = -0.2 }
    val gearAlittleRight = InstantCommand { gP = 0.2 }
    val stopGear = InstantCommand { gP = 0.0 }

    fun autoAim() {
        // FIX: Check if pose is valid before using it
        if (!DriveTrain.isPoseValid()) {
            if (!hasWarnedAboutPose) {
                ActiveOpMode.telemetry.addData("Turret Warning", "Pose not initialized, cannot auto-aim")
                hasWarnedAboutPose = true
            }
            turret.power = 0.0
            return
        }

        hasWarnedAboutPose = false // Reset warning flag

        // FIX: Use DriveTrain object properties instead of direct imports
        val mu = atan2(goalY - DriveTrain.currentY, goalX - DriveTrain.currentX)
        val deltaHeading = normalizeAngle(mu - DriveTrain.currentHeading)
        val clampedHeading = deltaHeading.coerceIn(-PI, PI)

        turretController.goal = KineticState(clampedHeading, 0.0)
        turret.power = turretController.calculate(KineticState(getYaw(), 0.0))
    }

    fun goToYaw(yaw: Double) {
        turretController.goal = KineticState(yaw, 0.0)
        turret.power = turretController.calculate(KineticState(getYaw(), 0.0))
    }

    fun getYaw(): Double {
        return normalizeAngle(turret.currentPosition * rpt)
    }

    fun normalizeAngle(angleRadians: Double): Double {
        var angle = angleRadians % (2.0 * PI)
        if (angle <= -PI) {
            angle += 2.0 * PI
        }
        if (angle > PI) {
            angle -= 2.0 * PI
        }
        return angle
    }

    // ========== ABSOLUTE ENCODER FUNCTIONS ==========

    // FIX: Safe null checks for all encoder functions
    fun getAbsolutePositionRatio(): Double {
        return turretEncoder?.let {
            it.voltage / it.maxVoltage
        } ?: 0.0
    }

    fun getAbsolutePositionDegrees(): Double {
        return getAbsolutePositionRatio() * 360.0
    }

    fun getAbsolutePositionRadians(): Double {
        return getAbsolutePositionRatio() * 2.0 * PI
    }

    fun getAbsoluteYaw(): Double {
        return normalizeAngle(getAbsolutePositionRadians() - encoderOffset)
    }

    fun calibrateAbsoluteEncoder() {
        turretEncoder?.let {
            encoderOffset = getAbsolutePositionRadians()
            ActiveOpMode.telemetry.addData("Encoder Calibrated", "Offset: $encoderOffset")
        } ?: run {
            ActiveOpMode.telemetry.addData("Calibration Failed", "Encoder not initialized")
        }
    }

    val calibrateEncoderCommand = InstantCommand {
        calibrateAbsoluteEncoder()
    }

    fun autoAimAbsolute() {
        // FIX: Check both encoder and pose validity
        if (turretEncoder == null) {
            autoAim() // Fall back to relative encoder
            return
        }

        if (!DriveTrain.isPoseValid()) {
            if (!hasWarnedAboutPose) {
                ActiveOpMode.telemetry.addData("Turret Warning", "Pose not initialized, cannot auto-aim")
                hasWarnedAboutPose = true
            }
            turret.power = 0.0
            return
        }

        hasWarnedAboutPose = false

        val mu = atan2(goalY - DriveTrain.currentY, goalX - DriveTrain.currentX)
        val deltaHeading = normalizeAngle(mu - DriveTrain.currentHeading)
        val clampedHeading = deltaHeading.coerceIn(-PI, PI)

        turretController.goal = KineticState(clampedHeading, 0.0)
        turret.power = turretController.calculate(KineticState(getAbsoluteYaw(), 0.0))
    }

    fun goToYawAbsolute(yaw: Double) {
        if (turretEncoder == null) {
            goToYaw(yaw)
            return
        }

        turretController.goal = KineticState(yaw, 0.0)
        turret.power = turretController.calculate(KineticState(getAbsoluteYaw(), 0.0))
    }
}