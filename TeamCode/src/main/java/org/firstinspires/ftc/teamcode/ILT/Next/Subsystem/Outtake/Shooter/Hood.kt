package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx


@Configurable

object Hood: Subsystem {

     val hS = ServoEx("hood")


     var hP = 0.0
        set(value) {
            // Clamp value between 0.0 and 1.0 whenever hP is set
            field = value.coerceIn(0.0, 1.0)
        }
    val FlapDown = InstantCommand { hP += 0.05 }
    val FlapUp = InstantCommand { hP -= 0.05 }
    val stopHood = InstantCommand{ hP = 0.0}

    fun getHoodPosition(){
      // if i got time add enum for this too
        // and make 4 positions
    }

    override fun periodic() {

        hS.position = hP
    }

    fun updatePosition(position: Double) {

        hP = position
    }
}