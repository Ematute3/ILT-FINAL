package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object Intake : Subsystem {

    val iMR = MotorEx("iMR")
    val iML = MotorEx("iML")
    val iM = MotorGroup(iML, iMR)

    @JvmField
    var iP = 0.0

    // FIX: Add state tracking
    enum class IntakeState {
        STOPPED,
        INTAKING,
        EJECTING,
        FEEDING  // When feeding to shooter
    }

    var state = IntakeState.STOPPED
        private set

    override fun periodic() {
        iM.power = iP

        // FIX: Add telemetry
        ActiveOpMode.telemetry.addData("Intake State", state)
        ActiveOpMode.telemetry.addData("Intake Power", iP)
    }

    // FIX: Update state when changing power
    val runIntake = InstantCommand {
        iP = 1.0
        state = IntakeState.INTAKING
    }

    val reverseIntake = InstantCommand {
        iP = -1.0
        state = IntakeState.EJECTING
    }

    val reverseIntakeSlow = InstantCommand {
        iP = -0.5
        state = IntakeState.EJECTING
    }

    val reverseIntakeVerySlow = InstantCommand {
        iP = -0.2
        state = IntakeState.EJECTING
    }

    val stopIntake = InstantCommand {
        iP = 0.0
        state = IntakeState.STOPPED
    }

    // FIX: Add feeding command for shooting sequence
    val feedShooter = InstantCommand {
        iP = 0.8
        state = IntakeState.FEEDING
    }

    // Helper function to check if intake is running
    fun isRunning(): Boolean {
        return state != IntakeState.STOPPED
    }
}