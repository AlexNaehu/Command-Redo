package frc.robot;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.hal.ThreadsJNI;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.SensorObject;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    
    public final static XboxController controller1 = new XboxController(0);

    public final static FlyWheel flyWheel = new FlyWheel();

    public static SensorObject sensor = new SensorObject();
    private UsbCamera cam;

    private double xPosition;
    private double yPosition;
    private double thetaPosition;
    


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -controller1.getLeftY(), //left and right is Y axis per roborio
                () -> -controller1.getLeftX(), //forward and back is X axis per roborio
                () -> -controller1.getRightX(), //left is turn left, right is turn right
                () -> !controller1.getBackButton(), //field oriented function unless pressed
                () ->  controller1.getYButton(), //Aimbot AprilTag
                () ->  controller1.getRightTriggerAxis())); //shoots flywheel

        configureButtonBindings();

        cam = CameraServer.startAutomaticCapture();
        cam.setFPS(15);
        cam.setResolution(320, 240);
        cam.setPixelFormat(PixelFormat.kMJPEG);
    }

    private void configureButtonBindings() {
        /*--------------------------------------------------------------------------
        *  Zero Heading - Manual Control (1)
        *-------------------------------------------------------------------------*/
        if(controller1.getAButton()){
          swerveSubsystem.zeroHeading();
        }

    }
    
    private boolean isBetween(double number, double lower, double upper){
        if(number>=lower&&number<=upper){
                return true;
        }
        else{
                return false;
        }
    }


    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory (Note: I can add interior waypoints, but everytime I need a new angle orientation, I need to concatinate trajecotries as trejectory is not modified in interior waypoints)
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.75, 0),
                        new Translation2d(0.75, -0.75)),
                new Pose2d(1.5, -0.75, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.5, -0.75, new Rotation2d(180)),
                List.of(
                        new Translation2d(0.75, -0.75),
                        new Translation2d(0.75, 0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        Trajectory trajectory3 = trajectory.concatenate(trajectory2);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory3,    //JUST CHANGED TO A CONCATENATED TRAJECTORY
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add additional commands (intake, flywheel, arm, aimbot, ect.)
        xPosition = swerveSubsystem.getPose().getX();
        yPosition = swerveSubsystem.getPose().getY();
        thetaPosition = swerveSubsystem.getPose().getRotation().getDegrees();

        if(isBetween(xPosition, 1.95, 2.05) && isBetween(yPosition, -0.95, -1.05)){
                //flyWheel.shoot(1);
        }
        else{
                //flyWheel.shoot(0);
        }
        if(xPosition >= 1.7){
                swerveSubsystem.stopModules();
        }
        
        // 6. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory3.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
