package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.KineticState
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret

@TeleOp(name = "Flywheel Test", group = "Test")
class FlywheelTest : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(
                DriveTrain,
                FlyWheel,
                Hood,
                Turret,
                Intake,
                ImprovedOuttake
            ),
            BindingsComponent,
            BulkReadComponent
        )
    }

    private lateinit var tele: JoinedTelemetry

    override fun onInit() {
        tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        DriveTrain.setAlliance(Alliance.RED)
        DriveTrain.initialize()
        Turret.initialize()
        ImprovedOuttake.initialize()

        follower.setStartingPose(Pose(144.0 - 36.0, 6.5, Math.PI / 2))

        telemetry.addLine("Flywheel Test Initialized")
        telemetry.addLine("Controls:")
        telemetry.addLine("  X - Spin flywheel")
        telemetry.addLine("  Y - Stop flywheel")
        telemetry.addLine("  Cross - Full shoot sequence")
        telemetry.addLine("  DPad Up - Zero turret encoder")
        telemetry.addLine("  RB/LB - Turret manual control")
        telemetry.addLine("  GP2 DPad Up/Down - Adjust velocity")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // Flywheel controls
        val spinFlywheel = Gamepads.gamepad1.x
        spinFlywheel.whenBecomesTrue { FlyWheel.spin.schedule() }

        val stopFlywheel = Gamepads.gamepad1.y
        stopFlywheel.whenBecomesTrue { FlyWheel.stop.schedule() }

        val shootSequence = Gamepads.gamepad1.cross
        shootSequence.whenBecomesTrue { FlyWheel.Shoot.schedule() }

        // Turret controls
        val zeroTurret = Gamepads.gamepad1.dpadUp
        zeroTurret.whenBecomesTrue { Turret.zeroMotor.schedule() }

        val turretRight = Gamepads.gamepad2.rightBumper
        turretRight.whenBecomesTrue { Turret.spinGearRight.schedule() }
        turretRight.whenBecomesFalse { Turret.stopGear.schedule() }

        val turretLeft = Gamepads.gamepad2.leftBumper
        turretLeft.whenBecomesTrue { Turret.spinGearLeft.schedule() }
        turretLeft.whenBecomesFalse { Turret.stopGear.schedule() }

        // Velocity adjustment
        val veloUp = Gamepads.gamepad2.dpadUp
        veloUp.whenBecomesTrue { FlyWheel.targetVelocity += 50 }

        val veloDown = Gamepads.gamepad2.dpadDown
        veloDown.whenBecomesTrue { FlyWheel.targetVelocity -= 50 }
    }

    override fun onUpdate() {
        tele.run {
            addLine("=== FLYWHEEL ===")
            addData("F1 Power", "%.3f".format(FlyWheel.f1.power))
            addData("F1 Velocity", "%.1f".format(FlyWheel.f1.velocity))
            addData("F2 Velocity", "%.1f".format(FlyWheel.f2.velocity))
            addData("Motor RPM", "%.0f".format(FlyWheel.motorRpm))
            addData("Target RPM", "%.0f".format(FlyWheel.targetVelocity * 60.0 / 28.0))
            addData("Target Velocity", "%.1f".format(FlyWheel.targetVelocity))
            addData("Flywheels On", FlyWheel.flywheelsOn)
            addData("At Speed", FlyWheel.isAtTargetVelocity())

            val currentState = KineticState(FlyWheel.f1.currentPosition.toDouble(), FlyWheel.f1.velocity)
            addData("Controller Output", "%.3f".format(
                FlyWheel.flywheelController.calculate(currentState)
            ))

            addLine("=== TURRET ===")
            addData("Turret gP", "%.2f".format(Turret.gP))
            addData("Turret Power", "%.3f".format(Turret.turret.power))
            addData("Turret Velocity", "%.1f".format(Turret.turret.velocity))
            addData("Turret Position", Turret.turret.currentPosition)
            addData("Turret Yaw", "%.2f°".format(Math.toDegrees(Turret.getYaw())))
            addData("Turret Goal", "%.2f°".format(
                Math.toDegrees(Turret.turretController.goal.position)
            ))

            addLine("=== INTAKE ===")
            addData("Intake Power", "%.2f".format(Intake.iP))
            addData("Intake State", Intake.state)

            addLine("=== HOOD ===")
            addData("Hood Position", "%.3f".format(Hood.hP))
            addData("Hood Servo", "%.3f".format(Hood.hS.position))

            addLine("=== TARGETING ===")
            addData("Goal X", "%.1f".format(ImprovedOuttake.goalX))
            addData("Goal Y", "%.1f".format(ImprovedOuttake.goalY))
            addData("Distance (Odom)", "%.1f".format(ImprovedOuttake.distanceToGoalOdometry))
            addData("Distance (LL)", ImprovedOuttake.distanceToGoalLimelight?.let {
                "%.1f".format(it)
            } ?: "N/A")
            addData("Distance Diff", "%.1f".format(ImprovedOuttake.distanceDifference))

            addLine("=== ROBOT POSE ===")
            addData("Pose Valid", DriveTrain.isPoseValid())
            addData("Current X", "%.1f".format(DriveTrain.currentX))
            addData("Current Y", "%.1f".format(DriveTrain.currentY))
            addData("Current Heading", "%.2f°".format(Math.toDegrees(DriveTrain.currentHeading)))
            addData("In Shoot Zone", DriveTrain.inShootZone())

            update()
        }
    }
}