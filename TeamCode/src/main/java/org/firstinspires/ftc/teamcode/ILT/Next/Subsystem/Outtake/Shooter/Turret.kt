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
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import kotlin.math.PI
import kotlin.math.atan2
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake.goalY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake.goalX
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret




object Turret: Subsystem {



     val turret = MotorEx("turret")

    private lateinit var turretEncoder: AnalogInput
    var encoderOffset = 0.0

    var gP = 0.0



    private val gearRatio = 3.62068965517
    //105/29

    @JvmField
    var autoTurret = true // Enables automatic aiming behavior when true.

    // PID coefficients for position control of the turret (tuned empirically).
    @JvmField
    var turretPID = PIDCoefficients(0.011, 0.0, 0.2)


    override fun initialize() {
        // Initialize absolute encoder using ActiveOpMode.hardwareMap
        // This name MUST match the name in Driver Hub configuration
        turretEncoder = ActiveOpMode.hardwareMap.get(AnalogInput::class.java, "encoder")

        // Set initial offset (calibrate turret to forward position)
        // This will be adjusted when you run calibrateAbsoluteEncoder command
        encoderOffset = 0.0
    }



    // Control system that uses the position PID to compute motor power based on goal vs current state.
    var turretController = controlSystem {
        posPid(turretPID)
    }

    private val ppr = 537.7 // The resolution of our motor encoder on the goBilda site
    // may have to change thisd based on the absolute encoder

    private val rpt = 2* PI /(ppr * gearRatio)


    override fun periodic() {
        if(autoTurret) {

            //manually auto aim with out encoder.
                autoAim()
            // auto aim with the LL
                //autoAimLL()
            // auto aim with the encoder
                //autoAimAbsolute()
        }


        ActiveOpMode.telemetry.run {
            addData("goal", turretController.goal.position)
            addData("turret Pos", getYaw())
        }
    }
    private fun autoAimLL(){
        LLTurret.toggleAutoAimLL
    }



    val zeroMotor = InstantCommand {
        turret.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    val spinGearLeft = InstantCommand { gP = 0.6 }
    val spinGearRight = InstantCommand { gP = -0.6 }
    val gearAlittleLeft = InstantCommand { gP = -0.2 }
    val gearAlittleRight = InstantCommand { gP = 0.2 }
    val stopGear = InstantCommand { gP = 0.0 }

    val mu2 = atan2(goalY - currentY, goalX - currentX)

    val deltaHeading2 = normalizeAngle(this.mu2 - currentHeading)



     fun autoAim() {
        // add the abosulte encode that we got
        val mu = atan2(goalY - currentY, goalX - currentX)


        val deltaHeading = normalizeAngle(mu - currentHeading)

        // change this based off degree with new wires anmd my mess up
        val clampedHeading = deltaHeading.coerceIn(-PI, PI)


        turretController.goal = KineticState(clampedHeading, 0.0)


        turret.power = turretController.calculate(KineticState(getYaw(), 0.0))
    }


    fun goToYaw(yaw:Double) {
        turretController.goal = KineticState(yaw, 0.0)
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

    fun getAbsolutePositionRatio(): Double {
        return turretEncoder.voltage / turretEncoder.maxVoltage
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
        encoderOffset = getAbsolutePositionRadians()
    }

    val calibrateEncoderCommand = InstantCommand {
        calibrateAbsoluteEncoder()
    }

    fun autoAimAbsolute() {
        val mu = atan2(goalY - currentY, goalX - currentX)
        val deltaHeading = normalizeAngle(mu - currentHeading)
        val clampedHeading = deltaHeading.coerceIn(-PI, PI)

        turretController.goal = KineticState(clampedHeading, 0.0)

        // Use absolute encoder for position feedback
        turret.power = turretController.calculate(KineticState(getAbsoluteYaw(), 0.0))
    }

    fun goToYawAbsolute(yaw: Double) {
        turretController.goal = KineticState(yaw, 0.0)
        turret.power = turretController.calculate(KineticState(getAbsoluteYaw(), 0.0))
    }

}



