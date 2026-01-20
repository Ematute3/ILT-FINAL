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
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.OuttakeNew
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret

@TeleOp(name = "Outtake Test", group = "Test")
class OuttakeTest: NextFTCOpMode() {

    init {
        // FIX: Add ALL required subsystems in correct order
        addComponents(
            SubsystemComponent(
                DriveTrain,      // Must be first
                limeLight,       // Second for vision
                Turret,          // Shooter components
                FlyWheel,
                Hood,
                Intake,
                LLAutoVelo,      // Vision processing
                LLTurret,        // Vision-based turret control
                ImprovedOuttake, // Coordination
                OuttakeNew       // Main controller
            ),
            BindingsComponent,
            BulkReadComponent
        )
    }

    var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    override fun onInit() {
        // FIX: Initialize all subsystems in correct order
        DriveTrain.setAlliance(Alliance.RED)
        DriveTrain.initialize()
        limeLight.initialize()
        Turret.initialize()
        LLAutoVelo.initialize()
        ImprovedOuttake.initialize()

        // Set starting pose
        follower.setStartingPose(Pose(144.0 - 36.0, 6.5, Math.PI / 2))

        telemetry.addLine("Outtake Test Initialized")
        telemetry.addLine("=== MODE CONTROLS ===")
        telemetry.addLine("A - Manual Mode")
        telemetry.addLine("B - Auto Mode (Odometry)")
        telemetry.addLine("X - Auto Mode (Limelight)")
        telemetry.addLine("Y - Idle Mode")
        telemetry.addLine()
        telemetry.addLine("=== MANUAL CONTROLS ===")
        telemetry.addLine("GP2 DPad Up/Down - Aim distance")
        telemetry.addLine("GP2 RB/LB - Turret rotation")
        telemetry.addLine("GP2 Right Trigger - Spin flywheel")
        telemetry.addLine("GP2 Left Trigger - Run intake")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // FIX: Proper mode switching with .schedule()
        Gamepads.gamepad1.a whenBecomesTrue {
            OuttakeNew.manualMode.schedule()
        }

        Gamepads.gamepad1.b whenBecomesTrue {
            OuttakeNew.autoModeManual.schedule()
        }

        Gamepads.gamepad1.x whenBecomesTrue {
            OuttakeNew.autoModeLL.schedule()
        }

        // FIX: Y button for idle
        Gamepads.gamepad1.y whenBecomesTrue {
            OuttakeNew.idleMode.schedule()
        }

        // Manual controls (work in manual mode)
        Gamepads.gamepad2.dpadUp whenBecomesTrue {
            ImprovedOuttake.aimUp.schedule()
        }

        Gamepads.gamepad2.dpadDown whenBecomesTrue {
            ImprovedOuttake.aimDown.schedule()
        }

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

        // Shooter controls
        Gamepads.gamepad2.rightTrigger.greaterThan(0.5) whenBecomesTrue {
            FlyWheel.spin.schedule()
        }  whenBecomesFalse {
            FlyWheel.stop.schedule()
        }

        // Intake controls
        Gamepads.gamepad2.leftTrigger.greaterThan(0.5) whenBecomesTrue {
            Intake.runIntake.schedule()
        } whenBecomesFalse {
            Intake.stopIntake.schedule()
        }

        // Emergency stop
        Gamepads.gamepad2.back whenBecomesTrue {
            OuttakeNew.idleMode.schedule()
            FlyWheel.stop.schedule()
            Intake.stopIntake.schedule()
        }

        // Test full auto shoot sequence
        Gamepads.gamepad2.start whenBecomesTrue {
            if (OuttakeNew.canUseAutoModes()) {
                ImprovedOuttake.fullAutoShootSequence.schedule()
            }
        }
    }

    override fun onUpdate() {
        tele.run {
            addLine("=== OUTTAKE STATUS ===")
            addData("Adjust Mode", OuttakeNew.adjustMode)
            addData("Turret Mode", OuttakeNew.turretMode)
            addLine()

            addLine("=== SYSTEM READY ===")
            addData("Pose Valid", DriveTrain.isPoseValid())
            addData("LL Ready", limeLight.isReady())
            addData("LL Has Target", limeLight.hasValidTarget)
            addData("Can Use Auto", OuttakeNew.canUseAutoModes())
            addData("Can Use LL", OuttakeNew.canUseLLModes())
            addLine()

            addLine("=== FLYWHEEL ===")
            addData("F1 Power", "%.3f".format(FlyWheel.f1.power))
            addData("Motor RPM", "%.0f".format(FlyWheel.motorRpm))
            addData("Target RPM", "%.0f".format(FlyWheel.targetVelocity * 60.0 / 28.0))
            addData("Flywheels On", FlyWheel.flywheelsOn)
            addData("At Speed", FlyWheel.isAtTargetVelocity())
            addLine()

            addLine("=== TURRET ===")
            addData("Turret Power", "%.3f".format(Turret.turret.power))
            addData("Turret Yaw", "%.2f°".format(Math.toDegrees(Turret.getYaw())))
            addData("Turret Goal", "%.2f°".format(
                Math.toDegrees(Turret.turretController.goal.position)
            ))
            addData("LL Aim Enabled", LLTurret.autoAimEnabled)
            addData("LL Aligned", LLTurret.isAligned)
            addLine()

            addLine("=== HOOD ===")
            addData("Hood Position", "%.3f".format(Hood.hP))
            addLine()

            addLine("=== INTAKE ===")
            addData("Intake Power", "%.2f".format(Intake.iP))
            addData("Intake State", Intake.state)
            addLine()

            addLine("=== TARGETING ===")
            addData("Goal Pos", "(%.1f, %.1f)".format(
                ImprovedOuttake.goalX, ImprovedOuttake.goalY
            ))
            addData("Robot Pos", "(%.1f, %.1f)".format(
                DriveTrain.currentX, DriveTrain.currentY
            ))
            addData("Distance (Odom)", "%.1f\"".format(ImprovedOuttake.distanceToGoalOdometry))
            addData("Distance (LL)", ImprovedOuttake.distanceToGoalLimelight?.let {
                "%.1f\"".format(it)
            } ?: "N/A")
            addData("In Shoot Zone", DriveTrain.inShootZone())
            addLine()

            addLine("=== MANUAL AIM ===")
            addData("Manual Aim Dist", ImprovedOuttake.manualAim)
            addData("Target Velo", "%.0f".format(ImprovedOuttake.targetVelo))
            addLine()

            addLine("=== LIMELIGHT ===")
            addData("TX", "%.2f°".format(limeLight.currentTx))
            addData("TY", "%.2f°".format(limeLight.currentTy))
            addData("TA", "%.2f%%".format(limeLight.currentTa))
            addData("Calculated RPM", "%.0f".format(LLAutoVelo.calculatedRPM))

            update()
        }
    }
}