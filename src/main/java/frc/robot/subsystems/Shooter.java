package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;


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
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {

   
    private SparkFlex shooterMotor = new SparkFlex(ShooterConstants.SHOOTER_CAN_ID, MotorType.kBrushless);
    private Servo actuatorMotor = new Servo(ShooterConstants.ACTUATOR_CAN_ID);

    public void setShooterSpeed(double distance_from_hub){
        
        double velocity_inches = (distance_from_hub)/
        (Math.sqrt(((2/ShooterConstants.GRAVITY) * (ShooterConstants.END_HEIGHT-ShooterConstants.STARTING_HEIGHT- Math.tan(ShooterConstants.HIGH_SHOOTER_ANGLE)*  distance_from_hub))) * Math.cos(ShooterConstants.HIGH_SHOOTER_ANGLE));

        double velocity_rpm = velocity_inches * (120/(4*Math.PI));
            System.out.println("speed" + velocity_rpm);
        
        double max_height = ShooterConstants.STARTING_HEIGHT - (Math.pow(velocity_inches * Math.sin(ShooterConstants.HIGH_SHOOTER_ANGLE), 2))/(2*ShooterConstants.GRAVITY);

        
        if (velocity_rpm < 6784 && velocity_rpm > 970 && max_height > 77) {
            double shooter_percentage = (velocity_rpm/6784) ; 

            shooterMotor.set(shooter_percentage);
        } else {
            shooterMotor.set(1.0);
        }
        
    }

   
    

    public void stopShooting(){
        shooterMotor.stopMotor();
    }

    public void setShooterangle(double shooterAngle){
       double actuatorPosition = ((85.94-shooterAngle)/6.88) / 5.512;
        System.out.println("actuator position" + actuatorPosition);
    

       if (actuatorPosition >= 0.2 && actuatorPosition <= 0.83 ){
       actuatorMotor.setPosition(actuatorPosition); }

       else {
        System.out.println("angle out of range");
       }
    }

    
}
 