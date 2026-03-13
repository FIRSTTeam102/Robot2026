package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;



import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Shooter extends SubsystemBase {

   
    private SparkFlex shooterMotor = new SparkFlex(ShooterConstants.SHOOTER_CAN_ID, MotorType.kBrushless);
    private RelativeEncoder shooterEncoder;
    private Servo actuatorMotor = new Servo(ShooterConstants.SERVO_CHANNEL);
    private SparkFlexConfig shooterConfig = new SparkFlexConfig();
    private SparkClosedLoopController shooterMotorClosedLoop;

    
     public Shooter() {
        shooterConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            
            .pid(ShooterConstants.SHOOTER_P_DEFAULT, ShooterConstants.SHOOTER_I_DEFAULT, ShooterConstants.SHOOTER_D_DEFAULT)
            .outputRange(-1.0, 1.0)
            .feedForward.kV(ShooterConstants.kV).kS(ShooterConstants.kS);

        shooterConfig.inverted(true);

        shooterConfig.encoder.velocityConversionFactor(1.0);
         
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotorClosedLoop = shooterMotor.getClosedLoopController();
        shooterEncoder = shooterMotor.getEncoder(); 
    }
    

    
     
    private final PIDController shooterPID = new PIDController(
        ShooterConstants.SHOOTER_P_DEFAULT,
        ShooterConstants.SHOOTER_I_DEFAULT, 
        ShooterConstants.SHOOTER_D_DEFAULT
    );//found with sysid

    @AutoLogOutput
    public double setShooterSpeed(double distance_from_hub){
        
        double velocity_inches = (distance_from_hub)/
        (Math.sqrt(((2/ShooterConstants.GRAVITY) * (ShooterConstants.STARTING_HEIGHT-ShooterConstants.END_HEIGHT- Math.tan(ShooterConstants.SHOOTER_ANGLE)*  distance_from_hub))) * Math.cos(ShooterConstants.SHOOTER_ANGLE));

        double velocity_rpm = velocity_inches * (120/(4*Math.PI));
            System.out.println("speed" + velocity_rpm);
        
        velocity_rpm = MathUtil.clamp(velocity_rpm, 970, 6784);

        double pidOutput = shooterPID.calculate(shooterRPM(),velocity_rpm);
        shooterMotor.set(-pidOutput);
        shooterPID.setSetpoint(pidOutput);


       
        


        /*if (velocity_rpm < 6784 && velocity_rpm > 970 ) {
            double shooter_percentage = (velocity_rpm/6784); 
                        System.out.println("speed" + shooter_percentage);

            shooterMotor.set(-shooter_percentage);
        } else {
            shooterMotor.set(-1.0); 
        }*/
        return velocity_rpm;
    }

    

    @AutoLogOutput
     public double shooterRPM() {
         return shooterEncoder.getVelocity();
     }

    public void startShooting(double rpm){
        double pidOutput = shooterPID.calculate(shooterRPM(),rpm);
        shooterMotor.set(pidOutput);
    }

    public void setShooterRPM(double rpm) {
        shooterMotorClosedLoop.setSetpoint(rpm, ControlType.kVelocity);
    }

   public double targetShooterPosition(double shooterAngle) {
    return (((((85.786-shooterAngle)/6.88) / 5.512))+0.296875)/1.5625;
   }

   public double getShooterPosition() {
    return actuatorMotor.getPosition();
   }
    

    public void stopShooting(){
        shooterMotor.stopMotor();
        shooterPID.reset();
    }

    public void setShooterangle(double shooterAngle){
       double actuatorPosition = (((((85.786-shooterAngle)/6.88) / 5.512))+0.296875)/1.5625;
    

       if (actuatorPosition >= 0.0 && actuatorPosition <= 1.0 ){
       actuatorMotor.setPosition(actuatorPosition); }

       else {
        System.out.println("angle out of range");
       }
    }

    public void setActuatorExtension(double distance) {
      actuatorMotor.setPosition((distance+0.296875)/1.5625);
    }
    
    private final SysIdRoutine sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> shooterMotor.setVoltage(voltage.in(Volts)),
            log -> log.motor("Shooter")
                .voltage(
                    Volts.of(
                        shooterMotor.getAppliedOutput()
                        * RobotController.getBatteryVoltage()
                    )
                )
                .angularVelocity(
                    RotationsPerSecond.of(
                        shooterEncoder.getVelocity() / 60.0
                    )
                ),
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
public void periodic() {
    if (Robot.ShooterP != null) {
        // shooterPID.setP(Robot.ShooterP.getDouble(ShooterConstants.SHOOTER_P_DEFAULT));
        // shooterPID.setI(Robot.ShooterI.getDouble(ShooterConstants.SHOOTER_I_DEFAULT));
        // shooterPID.setD(Robot.ShooterD.getDouble(ShooterConstants.SHOOTER_D_DEFAULT));

        shooterConfig.closedLoop.pid(
            Robot.ShooterP.getDouble(ShooterConstants.SHOOTER_P_DEFAULT),
            Robot.ShooterI.getDouble(ShooterConstants.SHOOTER_I_DEFAULT),
            Robot.ShooterD.getDouble(ShooterConstants.SHOOTER_D_DEFAULT)
            );
    }
}

}
 