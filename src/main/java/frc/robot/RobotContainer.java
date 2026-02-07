package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.25)
            .withRotationalDeadband(MaxAngularRate * 0.25)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // PS4 controller (replace to 0 if needed)
    private final CommandPS4Controller joystick = new CommandPS4Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // ---------------- INTAKE MOTOR ----------------
    // Set this to your real CAN ID from Phoenix Tuner
    private static final int INTAKE_CAN_ID = 30;

    // Intake speeds (tune)
    private static final double INTAKE_IN_PERCENT = 0.70;
    private static final double INTAKE_OUT_PERCENT = -0.50;

    private final TalonFX intakeMotor = new TalonFX(INTAKE_CAN_ID);
    private final DutyCycleOut intakeOut = new DutyCycleOut(0.0);

    // ---------------- LIMELIGHT DRIVE SETTINGS ----------------
    private static final int LL_AIM_PIPELINE = 0;

    private final PIDController aimPid = new PIDController(0.06, 0.0, 0.001);
    private final PIDController distancePid = new PIDController(0.6, 0.0, 0.0);

    private static final double DESIRED_TA = 2.0;

    private static final double MAX_AIM_RAD_PER_SEC = 2.5;
    private static final double MAX_AUTO_FWD_MPS = 2.0;

    public RobotContainer() {
        aimPid.setTolerance(1.0);
        aimPid.enableContinuousInput(-180, 180);

        distancePid.setTolerance(0.15);

        configureBindings();
    }

    private void configureBindings() {

        // ---------------- DEFAULT DRIVE (MANUAL) ----------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Cross = brake
        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // Circle = point wheels
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // SysId mappings (Share/Options + Triangle/Square)
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading on L1
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ---------------- INTAKE CONTROLS ----------------
        // Hold TRIANGLE to intake IN
        joystick.triangle().whileTrue(
            Commands.run(() -> intakeMotor.setControl(intakeOut.withOutput(INTAKE_IN_PERCENT)))
        ).onFalse(
            Commands.runOnce(() -> intakeMotor.setControl(intakeOut.withOutput(0.0)))
        );

        // Hold SQUARE to intake OUT (reverse)
        joystick.square().whileTrue(
            Commands.run(() -> intakeMotor.setControl(intakeOut.withOutput(INTAKE_OUT_PERCENT)))
        ).onFalse(
            Commands.runOnce(() -> intakeMotor.setControl(intakeOut.withOutput(0.0)))
        );

        // ---------------- LIMELIGHT: AIM ASSIST ----------------
        joystick.R1().whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(LL_AIM_PIPELINE);
                Limelight.setLedMode(0);
                aimPid.reset();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vx = joystick.getLeftY() * MaxSpeed;
                    double vy = -joystick.getLeftX() * MaxSpeed;

                    double omega = -joystick.getRightX() * MaxAngularRate;

                    if (Limelight.hasTarget()) {
                        double tx = Limelight.tx();
                        double turnCmd = aimPid.calculate(tx, 0.0);
                        turnCmd = MathUtil.clamp(turnCmd, -MAX_AIM_RAD_PER_SEC, MAX_AIM_RAD_PER_SEC);
                        omega = turnCmd;
                    }

                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(omega);
                })
            )
        );

        // ---------------- LIMELIGHT: DRIVE TO TARGET (OPTIONAL) ----------------
        joystick.R2().whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(0);
                Limelight.setLedMode(0);
                aimPid.reset();
                distancePid.reset();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vy = -joystick.getLeftX() * MaxSpeed;

                    double vx = joystick.getLeftY() * MaxSpeed;
                    double omega = -joystick.getRightX() * MaxAngularRate;

                    if (Limelight.hasTarget()) {
                        double ta = Limelight.ta();
                        double fwdCmd = distancePid.calculate(ta, DESIRED_TA);
                        fwdCmd = MathUtil.clamp(fwdCmd, -MAX_AUTO_FWD_MPS, MAX_AUTO_FWD_MPS);
                        vx = fwdCmd;

                        double tx = Limelight.tx();
                        double turnCmd = aimPid.calculate(tx, 0.0);
                        turnCmd = MathUtil.clamp(turnCmd, -MAX_AIM_RAD_PER_SEC, MAX_AIM_RAD_PER_SEC);
                        omega = turnCmd;
                    }

                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(omega);
                })
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}
