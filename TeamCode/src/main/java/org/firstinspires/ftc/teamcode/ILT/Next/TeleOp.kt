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
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Final-Teleop")
class TeleOP: NextFTCOpMode() {
    var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            // Register ALL subsystems including OuttakeNew and individual shooter subsystems
            SubsystemComponent(
                Intake,
                OuttakeNew,      // ✅ Added OuttakeNew
                ImprovedOuttake,
                DriveTrain,
                Hood,            // ✅ Added individual subsystems
                FlyWheel,
                Turret,
                LLTurret         // ✅ Added LLTurret so its periodic runs
            ),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    override fun onInit() {
        // Set alliance (you can make this configurable via gamepad later)
        DriveTrain.setAlliance(Alliance.RED)  // Change as needed

        // Optional: Set starting pose if needed
        // PedroComponent.follower?.setStartingPose(yourStartPose)
    }

    override fun onStartButtonPressed() {
        // ========== INTAKE CONTROLS (Gamepad 1) ==========
        Gamepads.gamepad1.rightTrigger.greaterThan(0.5) whenBecomesTrue Intake.runIntake whenBecomesFalse Intake.stopIntake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue Intake.reverseIntake whenBecomesFalse Intake.stopIntake

        // ========== MANUAL AIM ADJUSTMENT (Gamepad 2) ==========
        Gamepads.gamepad2.rightTrigger.greaterThan(0.5) whenBecomesTrue ImprovedOuttake.aimUp whenBecomesFalse ImprovedOuttake.stopAim
        Gamepads.gamepad2.leftTrigger.greaterThan(0.5) whenBecomesTrue ImprovedOuttake.aimDown whenBecomesFalse ImprovedOuttake.stopAim

        // ========== FLYWHEEL CONTROLS (Gamepad 2) ==========
        Gamepads.gamepad2.circle whenBecomesTrue FlyWheel.spin whenBecomesFalse FlyWheel.stop
        Gamepads.gamepad2.square whenBecomesTrue FlyWheel.stop
        Gamepads.gamepad2.x whenBecomesTrue FlyWheel.backOut whenBecomesFalse FlyWheel.stop

        // ========== TURRET MANUAL CONTROLS (Gamepad 2) ==========
        Gamepads.gamepad2.rightBumper whenBecomesTrue Turret.spinGearRight whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.leftBumper whenBecomesTrue Turret.spinGearLeft whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.dpadRight whenBecomesTrue Turret.gearAlittleRight whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.dpadLeft whenBecomesTrue Turret.gearAlittleLeft whenBecomesFalse Turret.stopGear

        // ========== HOOD CONTROLS (Gamepad 2) ==========
        Gamepads.gamepad2.dpadUp whenBecomesTrue Hood.FlapDown
        Gamepads.gamepad2.dpadDown whenBecomesTrue Hood.FlapUp

        // ========== MODE SWITCHING (Gamepad 1) ==========
        Gamepads.gamepad1.triangle whenBecomesTrue InstantCommand { DriveTrain.resetImu() }
        Gamepads.gamepad1.circle whenBecomesTrue OuttakeNew.manualMode        // ✅ Fixed
        Gamepads.gamepad1.square whenBecomesTrue OuttakeNew.autoModeManual    // ✅ Fixed
        Gamepads.gamepad1.x whenBecomesTrue OuttakeNew.autoModeLL             // ✅ Fixed
        Gamepads.gamepad1.rightBumper whenBecomesTrue OuttakeNew.idleMode     // ✅ Fixed
    }

    override fun onUpdate() {
        tele.run {
            addLine("========== OUTTAKE STATUS ==========")
            addData("Adjust Mode", OuttakeNew.adjustMode)
            addData("Turret Mode", OuttakeNew.turretMode)
            addLine()

            addLine("========== SHOOTER VALUES ==========")
            addData("Hood Position", "%.2f".format(Hood.hP))
            addData("Target Velocity", FlyWheel.targetVelocity)
            addData("Manual Aim Distance", "%.1f tiles".format(ImprovedOuttake.manualAim / 24.0))
            addLine()

            addLine("========== TURRET STATUS ==========")
            addData("Turret Yaw", "%.2f°".format(Math.toDegrees(Turret.getYaw())))
            addData("Absolute Yaw", "%.2f°".format(Math.toDegrees(Turret.getAbsoluteYaw())))
            addData("Encoder Offset", "%.3f".format(Turret.encoderOffset))
            addData("LL Auto-Aim", LLTurret.autoAimEnabled)
            addLine()

            addLine("========== DRIVETRAIN ==========")
            addData("X", "%.1f".format(DriveTrain.currentX))
            addData("Y", "%.1f".format(DriveTrain.currentY))
            addData("Heading", "%.1f°".format(Math.toDegrees(DriveTrain.currentHeading)))
            addData("In Shoot Zone", DriveTrain.inShootZone())

            update()
        }
    }
}