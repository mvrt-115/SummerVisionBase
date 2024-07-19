package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    public boolean isFieldOriented = true;
    public double NormalMaxSpeed = 4; //reference value, for when max speed is changed in robotcontainer
    public double MaxSpeed = NormalMaxSpeed; // 6 meters per second desired top speed
    public double NormalMaxAngularRate = 8.5 * Math.PI; //reference value 
    public double MaxAngularRate = 8.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    //private AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    private Pigeon2 pigeon = new Pigeon2(Constants.SwerveConstants.kPigeonId);

    /**
     * Heading correction stuff here
     */
    private boolean useHeadingCorrection = false;
    private double referenceHeading; // Reference heading for what current heading to
    private double heading; //Curr heading
    private double lastHeadingUpdateTime = Timer.getFPGATimestamp(); //Seconds

    private CommandXboxController joystick;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    
    private final SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
    public Swerve(CommandXboxController joystick, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        this.joystick = joystick;

        //gyro = new AHRS(SPI.Port.kMXP);

        // reset in new thread since gyro needs some time to boot up and we don't 
        // want to interfere with other code
        new Thread(() -> {
        try {
            Thread.sleep(1000);
            resetGyro();
        }
        catch (Exception e)
        {
            System.out.println("Reset Gyro Failed");
        }
        }).start(); 

        configurePathPlanner();
        setupModules();
    }

    public void setupModules(){
        CurrentLimitsConfigs cur_config = new CurrentLimitsConfigs();
        VoltageConfigs voltage_config = new VoltageConfigs().withPeakForwardVoltage(Constants.TalonConstants.kVoltageComp);
        cur_config.withSupplyCurrentLimit(Constants.TalonConstants.kDriveCurrentLimit);
        cur_config.withSupplyCurrentLimitEnable(true);
        
        CurrentLimitsConfigs cur_config_str = new CurrentLimitsConfigs();
        VoltageConfigs voltage_config_str = new VoltageConfigs().withPeakForwardVoltage(Constants.TalonConstants.kVoltageComp);
        cur_config.withSupplyCurrentLimit(Constants.TalonConstants.kSteerCurrentLimit);
        cur_config.withSupplyCurrentLimitEnable(true);
        
        for (SwerveModule m : Modules){
            m.getDriveMotor().getConfigurator().apply(
            new TalonFXConfiguration()
                .withCurrentLimits(cur_config));
            m.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
            m.getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
            // m.getSteerMotor().getConfigurator().apply(
            //     new TalonFXConfiguration().withCurrentLimits(cur_config_str)
            // );
        }
    }

    //updates current heading + resets reference heading if joystick is being turned
    public void updateCurrentHeading(double joystickRot){
        heading = getYaw();
        if (Math.abs(joystickRot) >= Constants.SwerveConstants.joystickThreshold){
            referenceHeading = heading;
            lastHeadingUpdateTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void periodic(){
        //Logger.recordOutput("penguin yaw", getYaw());
        //Logger.recordOutput("penguin joystick x", joystick.getLeftY());
        //Logger.recordOutput("penguin joystick y", joystick.getLeftX());
        //Logger.recordOutput("penguin chassis x velocities", getDesiredSpeeds(joystick.getLeftY(), joystick.getLeftX(), joystick.getRightX()).vxMetersPerSecond);
        //Logger.recordOutput("penguin chassis y velocities", getDesiredSpeeds(joystick.getLeftY(), joystick.getLeftX(), joystick.getRightX()).vyMetersPerSecond);
        //Logger.recordOutput("penguin chassis angle velocities", getDesiredSpeeds(joystick.getLeftY(), joystick.getLeftX(), joystick.getRightX()).omegaRadiansPerSecond);
        //Logger.recordOutput("penguin FIELD ORIENTED", isFieldOriented);
        updateCurrentHeading(joystick.getRightX());
    }

    public RobotCentric getSwerveCommand(){
        if (joystick.getLeftTriggerAxis() > Constants.SwerveConstants.joystickThreshold) {
            MaxSpeed = (1 - joystick.getLeftTriggerAxis() + 0.3) * NormalMaxSpeed;
            MaxAngularRate = (1 - joystick.getLeftTriggerAxis() + 0.3) * NormalMaxAngularRate;
        
        } else if (joystick.getRightTriggerAxis() > Constants.SwerveConstants.joystickThreshold) {
            MaxSpeed = (1 + 2.50 * joystick.getRightTriggerAxis()) * NormalMaxSpeed;
            MaxAngularRate = (1 + 2.50 * joystick.getRightTriggerAxis()) * NormalMaxAngularRate;
        }
        var speeds = getDesiredSpeeds(-joystick.getLeftY(), -joystick.getLeftX(), -(joystick.getRightX()));

        //Heading correction
        if (useHeadingCorrection) { applyHeadingCorrection(speeds); };

        return robotOriented.withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
    }

    /**
     * TODO: Test heading correction: might be (referenceHeading - heading), accounting for negative; idk
     */
    public void applyHeadingCorrection(ChassisSpeeds speeds){
        //Only change if current heading is off by > 0.5 degrees
        //NOTE: try to keep heading threshold very low, cuz otherwise it kinda is bad
        if(Math.abs(heading - referenceHeading) > Constants.SwerveConstants.headingThreshold){
            speeds = getDesiredSpeeds(-joystick.getLeftY(), -joystick.getLeftX(), 0.95 * -joystick.getRightX() + 0.05 * (heading - referenceHeading)/180);   
        }
    }

    /**
     * Return the field or robot relative chassis speeds depending on 
     * current status taking into account the robots angle from gyro
     * All passed in values are in range of -1 to 1
     * @param x positive = away from the driverstation
     * @param y positive = left when behind alliance wall
     * @param rot positive = counter-clockwise
     * @return new field oriented speeds
     */
    public ChassisSpeeds getDesiredSpeeds(double x, double y, double rot){
        if(!isFieldOriented){
            return new ChassisSpeeds(x * MaxSpeed, 
                                    y * MaxSpeed, 
                                    rot * MaxAngularRate);
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
        x * MaxSpeed, 
        y * MaxSpeed, 
        rot * MaxAngularRate, 
        Rotation2d.fromDegrees(getYaw()));
    }

    /**
     * Sets gyro offsets (for starting rotated, such as on the sides of the subwoofer)
     */
    public void setGyroOffset(double value){
        if(Constants.SwerveConstants.usingPigeon){
            pigeon.setYaw(value);
        }else{
            //figure out if this is the correct method
            //gyro.setAngleAdjustment(value);
        }
    }

    /**
     * Resets both the reference and the current heading 
     * Assuming "heading" isn't set to 0 here; if it's set to zero then the referenceHeading var is not needed
     */
    public void resetGyro(){
        if (Constants.SwerveConstants.usingPigeon){
            pigeon.reset();
        
        } else {
            //gyro.reset();
        }

        heading = getYaw();
        referenceHeading = heading;
    }

    @Override
    public Rotation3d getRotation3d(){
        if (Constants.SwerveConstants.usingPigeon){
            return super.getRotation3d();
        } 
        return null;
        //return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
    }

    /**
     * @return yaw of gyro, pos = CCW, in degrees
     */
    public double getYaw(){
        if (Constants.SwerveConstants.usingPigeon){
            return pigeon.getYaw().getValueAsDouble();
        } 
        return 0.0;
        //navx positive yaw is cw, so inverted
        //return -gyro.getYaw();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = Units.inchesToMeters(12.747);
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
        ()->this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative,  // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                        new PIDConstants(10, 0, 0),
                                        Constants.SwerveConstants.kSpeedAt12VoltsMps,
                                        driveBaseRadius,
                                        new ReplanningConfig()),
        ()->false, // Change this if the path needs to be flipped on red vs blue
        this);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
}