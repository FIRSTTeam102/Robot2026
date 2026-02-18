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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;



import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
    private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private Servo actuatorMotor = new Servo(ShooterConstants.SERVO_CHANNEL);

    private final PIDController shooterPID = new PIDController(0.00025, 0, 0.0);

    public void setShooterSpeed(double distance_from_hub){
        
        double velocity_inches = (distance_from_hub)/
        (Math.sqrt(((2/ShooterConstants.GRAVITY) * (ShooterConstants.STARTING_HEIGHT-ShooterConstants.END_HEIGHT- Math.tan(ShooterConstants.SHOOTER_ANGLE)*  distance_from_hub))) * Math.cos(ShooterConstants.SHOOTER_ANGLE));

        double velocity_rpm = velocity_inches * (120/(4*Math.PI));
            System.out.println("speed" + velocity_rpm);
        
        MathUtil.clamp(velocity_rpm, 970, 6784);

        double pidOutput = shooterPID.calculate(shooterRPM(),velocity_rpm);

        double feedforward = ShooterConstants.kS*Math.signum(velocity_rpm)+ShooterConstants.kV*velocity_rpm;

        shooterMotor.setVoltage(pidOutput+feedforward);

        /* 
        if (velocity_rpm < 6784 && velocity_rpm > 970 ) {
            double shooter_percentage = (velocity_rpm/6784); 
                        System.out.println("speed" + shooter_percentage);

            shooterMotor.set(-shooter_percentage);
        } else {
            shooterMotor.set(-1.0); 
        }
        */
    }

     public double shooterRPM() {
         return shooterEncoder.getVelocity();
     }

    public void startShooting(double velocity){
        shooterMotor.set(velocity);
    }

   
    

    public void stopShooting(){
        shooterMotor.stopMotor();
    }

    public void setShooterangle(double shooterAngle){
       double actuatorPosition = (((((85.786-shooterAngle)/6.88) / 5.512))+0.296875)/1.5625;
        System.out.println("actuator position" + actuatorPosition);
    

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


}
 