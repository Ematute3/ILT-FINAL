package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.iM
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.iP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.state
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel.flywheelsOn


object gate : Subsystem {

    val gate = ServoEx("gate")
    var gatePosition = 0.0

    override fun periodic() {
       gate.position = gatePosition
    }

    val gateOpen = InstantCommand {
       gatePosition = 0.0
        // gotta chagne the values
    }

    val gateClose = InstantCommand {
        // goitta change the values
        gatePosition = 1.0
    }
}