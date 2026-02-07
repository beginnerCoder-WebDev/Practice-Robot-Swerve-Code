package frc.robot;

import static edu.wpi.first.units.Units.*;

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

    // PS4 controller (change to 0)
    private final CommandPS4Controller joystick = new CommandPS4Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // ---------------- LIMELIGHT DRIVE SETTINGS ----------------
    // Pick the pipeline index you want for aiming (set in limelight.local)
    private static final int LL_AIM_PIPELINE = 0;

    // Aim Assist PID: rotates robot until tx -> 0
    // Start values: tune on the practice field.
    private final PIDController aimPid = new PIDController(0.06, 0.0, 0.001);

    // Drive-to-target PID using ta (area). Bigger ta = closer.
    // Only used in drive-to-target mode.
    private final PIDController distancePid = new PIDController(0.6, 0.0, 0.0);

    // Desired target area (ta) when you're at the right shooting/intake distance.
    // You MUST tune this for your camera mount + target type.
    private static final double DESIRED_TA = 2.0;

    // Limit outputs so it’s controllable
    private static final double MAX_AIM_RAD_PER_SEC = 2.5;    // clamp aiming turn
    private static final double MAX_AUTO_FWD_MPS = 2.0;       // clamp auto forward speed

    public RobotContainer() {
        // PID config
        aimPid.setTolerance(1.0);          // degrees
        aimPid.enableContinuousInput(-180, 180);

        distancePid.setTolerance(0.15);    // ta tolerance (tune)
        configureBindings();
    }

    private void configureBindings() {

        // ---------------- DEFAULT DRIVE (MANUAL) ----------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)              // forward/back
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)             // strafe
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // rotate
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

        // ---------------- LIMELIGHT: AIM ASSIST ----------------
        // Hold R1: keep driving with left stick, but auto-rotate to center target (tx -> 0)
        joystick.R1().whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(LL_AIM_PIPELINE);
                Limelight.setLedMode(0);   // use pipeline LED setting (or set 3 for force on)
                aimPid.reset();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vx = joystick.getLeftY() * MaxSpeed;
                    double vy = -joystick.getLeftX() * MaxSpeed;

                    double omega = -joystick.getRightX() * MaxAngularRate; // fallback to manual

                    if (Limelight.hasTarget()) {
                        double tx = Limelight.tx(); // degrees
                        // PID output (degrees -> turn rate command), then clamp
                        double turnCmd = aimPid.calculate(tx, 0.0); // want tx=0
                        turnCmd = MathUtil.clamp(turnCmd, -MAX_AIM_RAD_PER_SEC, MAX_AIM_RAD_PER_SEC);
                        omega = turnCmd; // override rotation with vision
                    }

                    return drive.withVelocityX(vx)
                                .withVelocityY(vy)
                                .withRotationalRate(omega);
                })
            )
        );

        // ---------------- LIMELIGHT: DRIVE TO TARGET (OPTIONAL) ----------------
        // Hold R2: robot auto-drives forward/back to DESIRED_TA, and auto-aims with tx.
        // (Strafe is still left stick so you can line up lanes while approaching.)
        joystick.R2().whileTrue(
            Commands.runOnce(() -> {
                Limelight.setPipeline(0);
                Limelight.setLedMode(0);
                aimPid.reset();
                distancePid.reset();
            }).andThen(
                drivetrain.applyRequest(() -> {
                    double vy = -joystick.getLeftX() * MaxSpeed;  // keep manual strafe if you want

                    double vx = joystick.getLeftY() * MaxSpeed;   // fallback manual
                    double omega = -joystick.getRightX() * MaxAngularRate;

                    if (Limelight.hasTarget()) {
                        // forward/back based on target area (ta)
                        double ta = Limelight.ta();
                        double fwdCmd = distancePid.calculate(ta, DESIRED_TA);
                        fwdCmd = MathUtil.clamp(fwdCmd, -MAX_AUTO_FWD_MPS, MAX_AUTO_FWD_MPS);
                        vx = fwdCmd;

                        // auto-aim rotation
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




