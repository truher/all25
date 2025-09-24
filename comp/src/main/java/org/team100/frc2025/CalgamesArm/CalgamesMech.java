package org.team100.frc2025.CalgamesArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.ElevatorArmWristKinematics;
import org.team100.lib.motion.Mech;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
//for arm motor
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;




public class CalgamesMech extends SubsystemBase{
    private final AngularPositionServo m_armServo;
    private final double m_armLengthM;
    private final double m_wristLengthM;
    private final ElevatorArmWristKinematics m_kinematics;
    private Config m_config;


    private static final double ARM_REDUCTION = 78; //number from om, from talon to output? (inspo from elevator code)


    public CalgamesMech(double armLength, int armMotorID, double wristLength, int wristMotorID, LoggerFactory parent, int canID){
        LoggerFactory log = parent.name("CalgamesArm");
        m_armLengthM = armLength;
        m_wristLengthM = wristLength;
        m_kinematics = new ElevatorArmWristKinematics(armLength, wristLength);
        // " in reality you wouldn't be able to just choose a default
        // configuration, you'd have to do something with the sensors." - joel in mech.java
        m_config = new Config(0.5, 0, 0); //grabbed this from mech.java, unclear what it does, i thought that config was for forward


        //all the below is stuff for the arm driving motor, taken from 2025 onseason climber code. not sure what most of it does
        Falcon6Motor armMotor = new Falcon6Motor(log, canID, MotorPhase.REVERSE, 50, 50,
        PIDConstants.makePositionPID(1),
        Feedforward100.makeArmPivot()); //not sure what this does, taken from climber class because arm is a falcon
        double inputOffset = 0; //0 for now, not known real value. this is just the 0 for the encoder, needs to be calibrated


        IncrementalProfile profile100 = new TrapezoidIncrementalProfile(0.5, 0.5, 0.05);
        ProfileReference1d ref = new IncrementalProfileReference1d(profile100, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 10, 0, 0, false, 0.05, 0.1);
        RotaryPositionSensor sensor = new AS5048RotaryPositionSensor( //this is for the encoder, probably wrong
            log, armMotorID, inputOffset, EncoderDrive.DIRECT); //what is input offset
        
        RotaryMechanism rotaryMechanism = new RotaryMechanism(
                log, armMotor, sensor, ARM_REDUCTION,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        m_armServo = new OutboardAngularPositionServo(log, rotaryMechanism, ref, feedback);


    }

    //simple methods grabbed from mech.java
    public static Mech make2025() {
        return new Mech(0.3, 0.1); //these values are prolly wrong, needs to be measured IRL
    }

    public double getArmLength() {
        return m_armLengthM;
    }

    public double getWristLength() {
        return m_wristLengthM;
    }

    public Config getConfig() {
        return m_config;
    }

    private void setConfig(Config config) {
        m_config = config;
    }


    public Command config(
            DoubleSupplier height,
            DoubleSupplier shoulder,
            DoubleSupplier wrist) {
        return run(() -> addConfig(
                height.getAsDouble(), shoulder.getAsDouble(), wrist.getAsDouble()));
    }

    /** Controls are cartesian axes. */
    public Command cartesian(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier r) {
        return run(() -> addCartesian(x.getAsDouble(), y.getAsDouble(), r.getAsDouble()));
    }

    /////////////////////////////////

    private void addConfig(double heightChange, double shoulderChange, double wristChange) {
        setConfig(new Config(
                m_config.shoulderHeight() + heightChange, //height here is the change in height requested
                m_config.shoulderAngle() + shoulderChange,//shoulder here is the change in shiulder angle requested
                m_config.wristAngle() + wristChange));//wrist here is the change in wrist angle requested
    }

    private void addCartesian(double x, double y, double r) {
        Transform2d t = new Transform2d(x, y, new Rotation2d(r)); //how much we want to change our goal point by (eg move up by 5, left by 3, rotate 20 degrees)
        Pose2d p = m_kinematics.forward(m_config); //runs forward kinematics to find out where we are before the move
        // Pose2d.add() method uses the pose frame so we don't use it.
        // We transform p in the global frame by adding components

        //finds our new goal point by adding the transformations, those from the first line (eg, 10+5)
        double x2 = p.getX() + t.getX();
        double y2 = p.getY() + t.getY();
        Rotation2d r2 = p.getRotation().plus(t.getRotation());
        Pose2d newP = new Pose2d(x2, y2, r2); //our new goal point
        Config c = m_kinematics.inverse(newP); //solves for the config to reach new goal point
        if (Double.isNaN(c.shoulderHeight()) //if the config fails, we quit
                || Double.isNaN(c.shoulderAngle())
                || Double.isNaN(c.wristAngle())) {
            System.out.println("skipping invalid config");
            return;
        }
        setConfig(c); //set the arm to the new config
    }
        /** Use a profile to set the position. */

    public void setArmtoConfig(){
        //this method will set the arm, wrist, and elevator
        
    }


    //all of the above is implementing the math, not actually moving the motors or mechansim
    //so need to add the like set position methods, like those in eleveator (use desmos kai)




    /*notes:
    calgamesarm class should be dumb, basically just figuring out how much to move things and moving them

    should make a seperate class for the needed trajecotries and figuring them out
    as part of that, we should make a thing to graph the changes in wrist, shoulder, elevator position (derive the trajectory)
        and make sure there are no discontiuites, those mean singulairty (i think)

    will make a seperate class that translates the points along the trajectory into simple poses accepatable by the dumb class

    */


}