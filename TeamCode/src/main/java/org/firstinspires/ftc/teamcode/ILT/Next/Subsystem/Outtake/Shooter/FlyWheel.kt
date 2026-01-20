package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds


object FlyWheel: Subsystem {

    val f1 = MotorEx("fly1")
    val f2 = MotorEx("fly2").reversed()

    // i needa tune this
    @JvmField
    var flywheelPID = PIDCoefficients(0.0033, 0.0, 0.0)

    @JvmField
    var flywheelFF = BasicFeedforwardParameters(1/2400.0, 0.0, 0.03)

    var flywheelController = controlSystem {
        velPid(flywheelPID)
        basicFF(flywheelFF)
    }

    @JvmField
    var targetVelocity = 0.0

    @JvmField
    var flywheelsOn = false

    @JvmField
    var velocityTolerance = 50.0

    var motorRpm: Double = 0.0

    // Add flag for manual control mode
    private var manualMode = false

    override fun periodic() {
        motorRpm = f1.velocity * 60.0 / 28.0

        // Only use controller if not in manual mode
        if (!manualMode) {
            // FIX: Create KineticState from motor's current position and velocity
            val currentState = KineticState(f1.currentPosition, f1.velocity)
            f1.power = flywheelController.calculate(currentState)
            f2.power = f1.power

            if (flywheelsOn) {
                flywheelController.goal = KineticState(0.0, targetVelocity)
            } else {
                flywheelController.goal = KineticState(0.0, 0.0)
            }
        }

        ActiveOpMode.telemetry.run {
            addData("targetVelo", targetVelocity)
            addData("Current RPM", motorRpm)
            addData("RPM target", targetVelocity * 60.0 / 28.0)
            addData("flywheel goal", flywheelController.goal)
            addData("At Speed", isAtTargetVelocity())
            addData("Manual Mode", manualMode)
        }
    }

    fun updatePid(velocity: Double) {
        targetVelocity = velocity
    }

    fun isAtTargetVelocity(): Boolean {
        return flywheelsOn && abs(f1.velocity - targetVelocity) < velocityTolerance
    }

    val spin = InstantCommand {
        manualMode = false
        flywheelsOn = true
    }

    val stop = InstantCommand {
        manualMode = false
        flywheelsOn = false
    }

    // FIX: Separate manual control from controller
    val backOutSlow = InstantCommand {
        manualMode = true
        flywheelsOn = false
        f1.power = -0.5
        f2.power = -0.5
    }

    val backOut = InstantCommand {
        manualMode = true
        flywheelsOn = false
        f1.power = -1.0
        f2.power = -1.0
    }

    val Shoot = SequentialGroup(
        spin,
        Delay(0.5.seconds),
        Intake.runIntake,
        Delay(0.5.seconds),
        stop
    )
}