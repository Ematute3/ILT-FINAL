package org.firstinspires.ftc.teamcode.next

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Final-Teleop")
class TeleOP : NextFTCOpMode() {
    private lateinit var tele: JoinedTelemetry

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                DriveTrain,
                limeLight,
                LLAutoVelo,
                Intake,
                ImprovedOuttake,  // Manages FlyWheel, Hood, Turret
                LLTurret,
                OuttakeNew
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

        DriveTrain.setAlliance(Alliance.RED)

        DriveTrain.initialize()
        limeLight.initialize()
        Turret.initialize()
        LLAutoVelo.initialize()
        ImprovedOuttake.initialize()

        telemetry.addLine("=== TELEOP INITIALIZED ===")
        telemetry.addLine("Alliance: ${DriveTrain.alliance}")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        // ========== INTAKE CONTROLS (Gamepad 1) ==========
        val intakeRun = Gamepads.gamepad1.rightTrigger.greaterThan(0.5)
        intakeRun.whenBecomesTrue { Intake.runIntake.schedule() }
        intakeRun.whenBecomesFalse { Intake.stopIntake.schedule() }

        val intakeReverse = Gamepads.gamepad1.leftTrigger.greaterThan(0.5)
        intakeReverse.whenBecomesTrue { Intake.reverseIntake.schedule() }
        intakeReverse.whenBecomesFalse { Intake.stopIntake.schedule() }

        // ========== MANUAL AIM ADJUSTMENT (Gamepad 2) ==========
        val aimUpTrigger = Gamepads.gamepad2.rightTrigger.greaterThan(0.5)
        aimUpTrigger.whenBecomesTrue { ImprovedOuttake.aimUp.schedule() }
        aimUpTrigger.whenBecomesFalse { ImprovedOuttake.stopAim.schedule() }

        val aimDownTrigger = Gamepads.gamepad2.leftTrigger.greaterThan(0.5)
        aimDownTrigger.whenBecomesTrue { ImprovedOuttake.aimDown.schedule() }
        aimDownTrigger.whenBecomesFalse { ImprovedOuttake.stopAim.schedule() }

        // ========== FLYWHEEL CONTROLS (Gamepad 2) ==========
        val flywheelSpin = Gamepads.gamepad2.circle
        flywheelSpin.whenBecomesTrue { FlyWheel.spin.schedule() }
        flywheelSpin.whenBecomesFalse { FlyWheel.stop.schedule() }

        val flywheelStop = Gamepads.gamepad2.square
        flywheelStop.whenBecomesTrue { FlyWheel.stop.schedule() }

        val flywheelBackOut = Gamepads.gamepad2.x
        flywheelBackOut.whenBecomesTrue { FlyWheel.backOut.schedule() }
        flywheelBackOut.whenBecomesFalse { FlyWheel.stop.schedule() }

        // ========== TURRET MANUAL CONTROLS (Gamepad 2) ==========
        val turretRight = Gamepads.gamepad2.rightBumper
        turretRight.whenBecomesTrue { Turret.spinGearRight.schedule() }
        turretRight.whenBecomesFalse { Turret.stopGear.schedule() }

        val turretLeft = Gamepads.gamepad2.leftBumper
        turretLeft.whenBecomesTrue { Turret.spinGearLeft.schedule() }
        turretLeft.whenBecomesFalse { Turret.stopGear.schedule() }

        val turretSlowRight = Gamepads.gamepad2.dpadRight
        turretSlowRight.whenBecomesTrue { Turret.gearAlittleRight.schedule() }
        turretSlowRight.whenBecomesFalse { Turret.stopGear.schedule() }

        val turretSlowLeft = Gamepads.gamepad2.dpadLeft
        turretSlowLeft.whenBecomesTrue { Turret.gearAlittleLeft.schedule() }
        turretSlowLeft.whenBecomesFalse { Turret.stopGear.schedule() }

        // ========== HOOD CONTROLS (Gamepad 2) ==========
        val hoodDown = Gamepads.gamepad2.dpadUp
        hoodDown.whenBecomesTrue { Hood.FlapDown.schedule() }

        val hoodUp = Gamepads.gamepad2.dpadDown
        hoodUp.whenBecomesTrue { Hood.FlapUp.schedule() }

        // ========== MODE SWITCHING (Gamepad 1) ==========
        val resetImu = Gamepads.gamepad1.triangle
        resetImu.whenBecomesTrue { DriveTrain.resetImu() }

        val manualMode = Gamepads.gamepad1.circle
        manualMode.whenBecomesTrue { OuttakeNew.manualMode.schedule() }

        val autoModeManual = Gamepads.gamepad1.square
        autoModeManual.whenBecomesTrue { OuttakeNew.autoModeManual.schedule() }

        val autoModeLL = Gamepads.gamepad1.x
        autoModeLL.whenBecomesTrue { OuttakeNew.autoModeLL.schedule() }

        val idleMode = Gamepads.gamepad1.rightBumper
        idleMode.whenBecomesTrue { OuttakeNew.idleMode.schedule() }

        // ========== EMERGENCY STOP (Gamepad 2 Back) ==========
        val emergencyStop = Gamepads.gamepad2.back
        emergencyStop.whenBecomesTrue {
            OuttakeNew.idleMode.schedule()
            FlyWheel.stop.schedule()
            Intake.stopIntake.schedule()
            Turret.stopGear.schedule()
        }

        // ========== FULL AUTO SHOOT (Gamepad 1 Start) ==========
        val fullAutoShoot = Gamepads.gamepad1.start
        fullAutoShoot.whenBecomesTrue {
            if (OuttakeNew.canUseAutoModes()) {
                ImprovedOuttake.fullAutoShootSequence.schedule()
            }
        }
    }

    override fun onUpdate() {
        tele.run {
            addLine("========== OUTTAKE STATUS ==========")
            addData("Adjust Mode", OuttakeNew.adjustMode)
            addData("Turret Mode", OuttakeNew.turretMode)
            addLine()

            addLine("========== SHOOTER VALUES ==========")
            addData("Hood Position", "%.2f".format(Hood.hP))
            addData("Target Velocity", "%.0f".format(FlyWheel.targetVelocity))
            addData("Current RPM", "%.0f".format(FlyWheel.motorRpm))
            addData("At Speed", FlyWheel.isAtTargetVelocity())
            addData("Manual Aim", "${ImprovedOuttake.manualAim}\" (%.1f tiles)".format(ImprovedOuttake.manualAim / 24.0))
            addLine()

            addLine("========== TURRET STATUS ==========")
            addData("Turret Yaw", "%.2f°".format(Math.toDegrees(Turret.getYaw())))
            addData("Absolute Yaw", "%.2f°".format(Math.toDegrees(Turret.getAbsoluteYaw())))
            addData("LL Auto-Aim", LLTurret.autoAimEnabled)
            addData("LL Aligned", LLTurret.isAligned)
            addLine()

            addLine("========== DRIVETRAIN ==========")
            addData("Pose Valid", DriveTrain.isPoseValid())
            addData("X", "%.1f".format(DriveTrain.currentX))
            addData("Y", "%.1f".format(DriveTrain.currentY))
            addData("Heading", "%.1f°".format(Math.toDegrees(DriveTrain.currentHeading)))
            addData("In Shoot Zone", DriveTrain.inShootZone())
            addLine()

            addLine("========== LIMELIGHT ==========")
            addData("Ready", limeLight.isReady())
            addData("Has Target", limeLight.hasValidTarget)
            if (limeLight.hasValidTarget) {
                addData("TX", "%.2f°".format(limeLight.currentTx))
                addData("Distance", LLAutoVelo.distanceToGoal?.let { "%.1f\"".format(it) } ?: "N/A")
            }

            update()
        }
    }
}