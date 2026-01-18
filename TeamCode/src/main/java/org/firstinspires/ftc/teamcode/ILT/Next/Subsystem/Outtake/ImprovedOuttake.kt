package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.OuttakeMode

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel.spin
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel.stop
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood.hP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood.hS
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.autoTurret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.gP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.turret
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.seconds


object ImprovedOuttake: SubsystemGroup(FlyWheel, Hood, Turret){




    var goalX = 0.0

    val goalY = 144-8.0

    var mode: OuttakeMode = OuttakeMode.IDLE

    override fun initialize() {

        goalX = if (DriveTrain.alliance == Alliance.RED) {
            144-6.0
        } else {
            6.0
        }
    }



    // got to use these in teleOP



    val distM: Double = sqrt((goalX-currentX).pow(2) + (goalY-currentY).pow(2))
    val distLL: Double = LLAutoVelo.distanceToGoal!!
    val distDiff = distM - distLL
    val values: DoubleArray = Aimbot.getValues(distM)
    val values2: DoubleArray = Aimbot.getValues(distLL)

    fun autoHoodFlyLL(){
    Hood.updatePosition(values2[0] + 0.06)
    FlyWheel.updatePid(values2[1] + 100)
    }
    fun autoHoodFlyManual() {

        Hood.updatePosition(values[0] + 0.06)
        FlyWheel.updatePid(values[1] + 100)
    }

    fun autoShoot() {

        if(DriveTrain.inShootZone()) {
            // TODO: Build command chain to spin up flywheels, set hood, align turret, and fire.
            Hood.updatePosition(values[0] + 0.06)  // Hood offset tweak
            FlyWheel.updatePid(values[1] + 100)
            FlyWheel.Shoot
        }
    }
     //ManualAim

    var targetVelo = 0.0
   var manualAim = 0
    val aimUp = InstantCommand { manualAim += 12 }
    val aimDown = InstantCommand { manualAim -= 12 }
    val stopAim = InstantCommand { manualAim = 0}
    @JvmField var canSpin = true
    fun manualAim() {

        if ( mode == OuttakeMode.MANUAL_ADJUST) {
                   aimDistance()
                  hS.position = hP
                  turret.power = gP
              }



    }
    fun aimDistance() {
        if (canSpin) {
            when (manualAim) {
                12 -> targetVelo = 835.0
                24 -> targetVelo = 862.0
                36 -> targetVelo = 844.0
                48 -> targetVelo = 848.0
                60 -> targetVelo = 908.0
                72 -> targetVelo = 1025.0
                84 -> targetVelo = 1165.0
                96 -> targetVelo = 1230.0
                108 -> targetVelo = 1070.0
                120 -> targetVelo = 1112.0
                132 -> targetVelo = 1150.0
                144 -> targetVelo = 1250.0
                else -> targetVelo = 0.0
            }
        }


        when (manualAim) {
            12 -> hP = 0.81
            24 -> hP = 0.93
            36 -> hP = 0.71
            48 -> hP = 0.6
            60 -> hP = 0.62
            72 -> hP = 0.65
            84 -> hP = 0.7
            96 -> hP = 0.7
            108 -> hP = 0.42
            120 -> hP = 0.43
            134 -> hP = 0.44
            146 -> hP = 0.45
            else -> hP = 0.0
        }


        if (manualAim > 146) manualAim = 146
        else if (manualAim < 12) manualAim = 12


        if (manualAim % 12 != 0) manualAim -= manualAim % 12
    }


}
