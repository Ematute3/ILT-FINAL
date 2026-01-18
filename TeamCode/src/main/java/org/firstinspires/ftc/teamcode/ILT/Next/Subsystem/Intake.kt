package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import kotlin.math.abs

@Configurable
object Intake : Subsystem {

    val iMR = MotorEx("iMR")
    val iML = MotorEx("iML")
    val iM = MotorGroup(iML, iMR)


    @JvmField
    var iP = 0.0


    override fun periodic() {

        iM.power = iP
    }



    val runIntake = InstantCommand {
        iP = 1.0
    }

    val reverseIntake = InstantCommand {
        iP = -1.0
    }

    val reverseIntakeSlow = InstantCommand {
        iP = -0.5
    }

    val reverseIntakeVerySlow = InstantCommand {
        iP = -0.2
    }

    val stopIntake = InstantCommand {
        iP = 0.0
    }
}

