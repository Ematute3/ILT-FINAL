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
class FlywheelTest: NextFTCOpMode() {

    init {
        // FIX: Add all required subsystems
        addComponents(
            SubsystemComponent(
                DriveTrain,      // Required for ImprovedOuttake
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

    var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    override fun onInit() {
        // FIX: Initialize DriveTrain first
        DriveTrain.setAlliance(Alliance.RED)
        DriveTrain.initialize()

        // Initialize other subsystems
        Turret.initialize()
        ImprovedOuttake.initialize()

        // Set starting pose for Pedro
        follower.setStartingPose(Pose(144.0 - 36.0, 6.5, Math.PI / 2))

        telemetry.addLine("Flywheel Test Initialized")
        telemetry.addLine("Controls:")
        telemetry.addLine("  X - Spin flywheel")
        telemetry.addLine("  Y - Stop flywheel")
        telemetry.addLine("  Cross - Full shoot sequence")
        telemetry.addLine("  DPad Up - Zero turret encoder")
        telemetry.addLine("  RB/LB - Turret manual control")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // FIX: Use .schedule() for all commands
        Gamepads.gamepad1.x whenBecomesTrue { FlyWheel.spin.schedule() }
        Gamepads.gamepad1.y whenBecomesTrue { FlyWheel.stop.schedule() }
        Gamepads.gamepad1.cross whenBecomesTrue { FlyWheel.Shoot.schedule() }
        Gamepads.gamepad1.dpadUp whenBecomesTrue { Turret.zeroMotor.schedule() }

        Gamepads.gamepad2.rightBumper whenBecomesTrue {
            Turret.spinGearRight.schedule()
        } whenBecomesFalse {
            Turret.stopGear.schedule()
        }

        Gamepads.gamepad2.leftBumper whenBecomesTrue {
            Turret.spinGearLeft.schedule()
        } whenBecomesFalse {
            Turret.stopGear.schedule()
        }

        // FIX: Add velocity adjustment controls
        Gamepads.gamepad2.dpadUp whenBecomesTrue {
            FlyWheel.targetVelocity += 50
        }
        Gamepads.gamepad2.dpadDown whenBecomesTrue {
            FlyWheel.targetVelocity -= 50
        }
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

            // FIX: Create state from current values instead of accessing .state
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
            // FIX: Use computed properties correctly
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