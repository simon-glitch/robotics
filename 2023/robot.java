// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.stream.Stream;

// don't need to import java.lang.*;
// import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MoveAction;
// import org.w3c.dom.Text;

import edu.wpi.first.wpilibj      .TimedRobot;
import edu.wpi.first.wpilibj      .Joystick  ;
import edu.wpi.first.wpilibj      .drive.DifferentialDrive      ;
import edu.wpi.first.wpilibj      .smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable        ;
import edu.wpi.first.networktables.NetworkTableEntry   ;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;

/**
 * import edu.wpi.first.wpiutil.net.PortForwarder;
 */
/**
 * The VM is configured to automatically run this class,
 * and to call the functions corresponding to each mode,
 * as described in the TimedRobot documentation.
 * If you change the name of this class or
 * the package after creating this project,
 * you must also update the
 * build.gradle file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the
   * robot is first started up and
   * should be used for any
   * initialization code.
   */
   
  // tilting angle of the limelight, in degrees
  private static double limelightTilt = 35;
  // height of the limelight, in inches
  // = {height of the robot base, in inches} + cos(limelightTilt) * {position of the limelight, going up the arm's support beam, in inches}
  private static double limelightHeight = ?? + Math.cos(limelightTilt * Math.PI/180) * ??;
  
  // motors and driveTrain (DifferentialDrive)
  private CANSparkMax  leftFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax   leftBackMotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax  rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);
  /**
   * armMotor is for arm rotation
   */
  private CANSparkMax        armMotor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax  armExtendMotor = new CANSparkMax(6, MotorType.kBrushless);
  
  private DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  // encoders
  // encoder
  private RelativeEncoder  leftFrontEncoder;
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder        armEncoder;
  
  
  // == turn and power constants
  private static double constantDeadband          = 0.1 ;
  private static double constantMotorMax          = 0.5 ;
  private static double constantProportionPower   = 1   ;
  private static double constantProportionTurn    = 0.05;
  private static double maxPower = constantProportionPower * constantMotorMax;
  private static double maxTurn  = constantProportionTurn  * constantMotorMax;
  
  // Joystick
  private Joystick joy1 = new Joystick(0);
  
  // turn and power
  private double power = 0;
  private double turn  = 0;
  private void getJoystickCoords(){
    power = joy1.getRawAxis(1);
    turn = -joy1.getRawAxis(0);
    // note to self: there are 4 axis; axis 2 is not very useful; axis 3 is very easy to control, but requires another hand
    SmartDashboard.putNumber("raw joystick power", power);
    SmartDashboard.putNumber("raw joystick turn" , turn );
  }
  private void driveTheRobot(){
    power = Math.signum(power) * Math.min(Math.abs(power), 1);
    turn  = Math.signum(turn ) * Math.min(Math.abs(turn ), 1);
    double appliedPower = power * maxPower;
    double appliedTurn  = turn  * maxTurn ;
    SmartDashboard.putNumber("applied power", appliedPower);
    SmartDashboard.putNumber("applied turn" , appliedTurn );
    SmartDashboard.putNumber("max power", maxPower);
    SmartDashboard.putNumber("max turn" , maxTurn );
    drive.arcadeDrive(appliedTurn, appliedPower);
  } 
  
  
  
  // == network table code for the limelight
  private NetworkTable   table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private double llx = 0;
  private double lly = 0;
  private double lla = 0;
  /**
   * (periodically) read values from the limelight's network table
   * @param doLog: whether we should post the limelight's values to Smart Dashboard; doLog = true causes values to be logged
   */
  private void readFromLimelight(boolean doLog){
    /* Simon's code to display entire network table
      | NetworkTableInstance tabinst = NetworkTableInstance.getDefault();
      | SmartDashboard.setNetworkTableInstance(tabinst);
    */
    
    table = NetworkTableInstance.getDefault().getTable("limelight-bacon");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    // read values from limelight periodically
    llx = tx.getDouble(0.0);
    lly = ty.getDouble(0.0);
    lla = ta.getDouble(0.0);
    // post from limelight to smart dashboard periodically
    SmartDashboard.putNumber("llx", llx);
    SmartDashboard.putNumber("lly", lly);
    SmartDashboard.putNumber("lla (area)", lla);
  }
  
  
  class HeightGuesser{
    /**
     * Make sure Aty and Bty are in degrees
     */
    private double calculateHeight(double Aty, double Bty, double ltilt, double Cx){
      Aty -= limelightTilt;
      Bty -= limelightTilt;
      Aty *= Math.PI / 180;
      Bty *= Math.PI / 180;
      return Math.sin(Bty) / Math.sin(Bty - Aty) * Cx;
    }
    public double travelDistance = 15; // half a foot
    public double encoderStart   =  0;
    public double encoderEnd     =  0;
    public double tyStart        =  0;
    public double tyEnd          =  0;
    public double measurement    =  0;
    public boolean isDoneTravelling = true;
    public void measuerInit(){
      if(isDoneTravelling){
        isDoneTravelling = false;
        encoderStart = leftFrontEncoder.getPosition();
        tyStart = ty.getDouble(0.0);
      }
    }
    public void measurePeriodic(){
      if(
        !this.isDoneTravelling &&
        (leftFrontEncoder.getPosition() - this.encoderStart >= this.travelDistance)
      ){
        this.isDoneTravelling = true;
        this.encoderEnd = leftFrontEncoder.getPosition();
        this.tyEnd      = ty.getDouble(0.0);
        double Cx = this.encoderEnd - this.encoderStart;
        // ATy = tyStart;
        // BTy = tyEnd  ;
        this.measurement = this.calculateHeight(
          this.tyStart ,
          this.tyEnd   ,
          limelightTilt,
          Cx
        );
        SmartDashboard.putNumber("ll height", this.measurement);
      }
    }
    public void horizontal_distance(goalHeight){
      // remember, ty is in degrees and limelightTilt are both in degrees
      
      double angleDiff;
      double distance ;
      
      angleDiff  = ty - limelightTilt;
      angleDiff /= Math.PI;
      distance   = (goalHeight - limelightHeight) / Math.tan(angleDiff)
      
      return distance;
      
    }
    
    
  }
  
  private HeightGuesser heightGuesser = new HeightGuesser();
  /*
    private double currentEncoderStartValueForVisionBasedRuler = 0;
    private double currentEncoder__EndValueForVisionBasedRuler = 0;
    private double currentTy_____StartValueForVisionBasedRuler = 0;
    private double currentTy_______EndValueForVisionBasedRuler = 0;
    private double                    minimumDistanceForMeasurement = 20; // half a foot
    private double previousEcnoderVisionBasedRulerHeightMeasurement = 0 ;
    private boolean heightGuesser.isDoneTravelling = true;
  */
  
  
  
  // count revolutions while spinning
  private static int llTurnFrameWavelength = 10;
  private int llTurnFrameIndex = 0;
  private boolean[] objectWasInView = new boolean[llTurnFrameWavelength];
  private boolean objectInView = false;
  private int objectSeenCount = 0;
  
  
  /*
  private double kg = 0.05;
  private double ks = 0.05;
  private double kv = 0.05;
  private double ka = 0.0;
  */
  
  private double driveGearRatio     = 10.71;
  private double wheelDiameter      = 6;
  private double wheelCircumference = 6 * Math.PI;
  
  private class PID{
    public double kP = 0.05;
    public double kI = 0.05;
    public double kD = 0.05;
    public double kOffset = 0;
    public double kFactor = 1;
    public double kiLimit = 1.0;
    public double kexp = 1.5;

    public double currPos = 0;
    public double setPoint = 0;
    public double errorSum;
    public double errorRate;
    public double prevTimestamp = Timer.getFPGATimestamp();
    public double currTimestamp = Timer.getFPGATimestamp();
    public double dt;
    public double error;
    public double lastError;
    
    public boolean avoidJerk;
    
    /**
     * set the constants on this PID controller
     * @param p: value of kP (proportional constant) - should be positive
     * @param i: value of kI (integral     constant) - should be small
     * @param d: value of kD (derivative   constant) - should be negative
     */
    public void setup(double p, double i, double d, double conversionFactor, double offset){
      kP = p;
      kI = i;
      kD = d;
      kOffset = offset;
      kFactor = conversionFactor;
      
    }
    
    public void evolvePID(double expSign, boolean doEvolveP, boolean doEvolveI, boolean doEvolveD){
      double mult = Math.pow(kexp, expSign * dt);
      SmartDashboard.putNumber("expSign", expSign);
      SmartDashboard.putNumber("mult"   , mult   );
      if(doEvolveP){
        kP *= mult;
      }
      if(doEvolveI){
        kI *= mult;
      }
      if(doEvolveD){
        kD *= mult;
      }
    }
    
    /**
     * set a new setPoint for this PID controller
     * @param newPoint is the new setPoint
     */
    public void controlInit(double newPoint){
      kOffset  = currPos ;
      setPoint = newPoint;
    }
    
    /**
     * set a new setPoint for this PID controller
     * @param newPoint is the new setPoint
     * @param doAvoidJerk use doAvoidJerk: true in order to avoid jerk when the PID starts
     */
    public void controlInit(double newPoint, boolean doAvoidJerk){
      kOffset   = currPos   ;
      setPoint  = newPoint  ;
      avoidJerk = doAvoidJerk;
      // should probably reset error sum too
      if(avoidJerk){
        errorSum = 0;
      }
    }
    
    public void updateTime(){
      prevTimestamp = currTimestamp;
      currTimestamp = Timer.getFPGATimestamp();
      dt = currTimestamp - prevTimestamp;
    }
    public void controlPeriodic(){
      
      // get sensor position
      currPos  = rightFrontEncoder.getPosition();
      currPos -= kOffset;
      currPos *= kFactor;
      
      // calculations
      lastError = error;
      error = setPoint - currPos;
      
      // keep track of time!
      updateTime();
      // check to make sure Timer is working
      if(dt <= 0){
        SmartDashboard.putNumber("problem: dt=", dt);
      }
      
      if (Math.abs(error) < kiLimit) {
        errorSum += error * dt;
      }
      errorSum *= 0.9;
      
      errorRate = (error - lastError) / dt;
      if(avoidJerk){
        avoidJerk = false;
        errorRate = 0;
      }
      
      // actual PID formula
      double driveSpeed = kP * error + kI * errorSum + kD * errorRate;
      
      power = driveSpeed;
      turn = 0;
      
    }
    
    public void zero(){
      controlInit(0);
      errorSum = 0;
      prevTimestamp = 0;
      lastError = 0;
    }
    
    public void log(String base){
      SmartDashboard.putNumber(base +" P.", kP);
      SmartDashboard.putNumber(base +" I.", kI);
      SmartDashboard.putNumber(base +" D.", kD);
      
      SmartDashboard.putNumber(base +" c pos" , currPos );
      SmartDashboard.putNumber(base +" set p" , setPoint);
      SmartDashboard.putNumber(base +" offset", kOffset );
      SmartDashboard.putNumber(base +" conv f", kFactor );
      
      SmartDashboard.putNumber(base +" err"    , error    );
      SmartDashboard.putNumber(base +" errSum" , errorSum );
      SmartDashboard.putNumber(base +" errRate", errorRate);
    }
  }
  
  /*
  armUp = (
    kg * Mathcos(angle) +
    ks * sgn(\dot{\theta}) +
    kv * angleD1 +
    ka * angleD2
  );
  */
  
  // all of our PIDs
  private PID         drivePID = new PID();
  private PID          turnPID = new PID();
  private PID           armPID = new PID();
  private PID     armExtendPID = new PID();
  private PID     limelightPID = new PID();
  private PID chargeStationPID = new PID();
  private PID rampPID = chargeStationPID;
  
  private boolean button[12];

  private void updateButtons() {
    for(int i = 0; i < button.length; i++){
      button[i]  = joy1.getRawButton(i) ;
    }
  }
  
  @Override
  public void robotInit() {
    
    // allow for slight movement in joystick to be set to zero
    drive.setDeadband(constantMotorMax * constantDeadband);
    
    //start the camera
    CameraServer.startAutomaticCapture();
    
    readFromLimelight(false);
    
    
    // set the motors opposite for turning
     leftFrontMotor.setInverted(true);
    rightFrontMotor.setInverted(true);
    
    // have the back wheels follow the front wheels
     leftBackMotor.follow(leftFrontMotor );
    rightBackMotor.follow(rightFrontMotor);
    
    /**
     * In order to read encoder values an encoder object is created using the
     * getEncoder() method from an existing CANSparkMax object
     */
     leftFrontEncoder =  leftFrontMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    
    resetEncoders();
  }
  
  public void resetEncoders(){
    // reset them evil encoders
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    
    drivePID.setup(
      0.05 ,
      0.025,
      -0.05,
      wheelCircumference / driveGearRatio,
      0
    );
    turnPID.setup(
      0.05 ,
      0.025,
      -0.05,
      wheelCircumference / driveGearRatio,
      0
    );
    armPID.setup(
      0.05 ,
      0.025,
      -0.05,
      wheelCircumference / driveGearRatio,
      0
    );
    armExtendPID.setup(
      0.05 ,
      0.025,
      -0.05,
      wheelCircumference / driveGearRatio,
      0
    );
    limelightPID.setup(
      0.05 ,
      0.025,
      -0.05,
      wheelCircumference / driveGearRatio,
      0
    );
    chargeStationPID.setup(
      0.05 ,
      0.025,
      -0.05,
      wheelCircumference / driveGearRatio,
      0
    );
    
            drivePID.zero();
             turnPID.zero();
              armPID.zero();
        armExtendPID.zero();
        limelightPID.zero();
    chargeStationPID.zero();
  }

  @Override
  public void robotPeriodic() {
  }
  
  @Override
  public void autonomousInit() {
    resetEncoders();
  }
  
  @Override
  public void autonomousPeriodic() {
    // just getting some buttons...
    updateButtons();
    // usePID();
    driveTheRobot();
    
    double kexpSign = joy1.getRawAxis(3);
    
  }
  
  @Override
  public void teleopInit() {
    resetEncoders();
    
    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    /**
     * for (int port = 5800; port <= 5805; port++) {
     * PortForwarder.add(port, "limelight.local", port);
    **/
    
  }

  @Override
  public void teleopPeriodic() {
    updateButtons();
    // logButtons();
    
    getJoystickCoords();

    // some encoder   code is at the top of the robot class
    // some limelight code is at the top of the robot class
    
    /*
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx    = table.getEntry("tx");
    ty    = table.getEntry("ty");
    ta    = table.getEntry("ta");
    */
    
    // read values periodically
    readFromLimelight(true);
    
    // log height guesser's values
    SmartDashboard.putNumber("start"   , heightGuesser.encoderStart);
    SmartDashboard.putNumber("end"     , heightGuesser.encoderEnd  );
    SmartDashboard.putNumber("start ty", heightGuesser.tyStart     );
    SmartDashboard.putNumber("end ty"  , heightGuesser.tyEnd       );
    
    
    if(button2){
      heightGuesser.measuerInit();
    }
    if(button1){
      /*
       * do stuff with aiming
       */
      drivePID.controlPeriodic();
      
    }
    else{
      drivePID.updateTime();
    }
    
    heightGuesser.measurePeriodic();
    
    double expSign = joy1.getRawAxis(3);
    
    
    // get joystick command
    if (button7) {
      drivePID.controlInit(24);
    }
    if (button8) {
      drivePID.controlInit(-24);
    }
    
    drivePID.evolvePID(expSign, button3, button4, button5);
    drivePID.log("drive");
    
    
    driveTheRobot();
    
    
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Le Enc Pos",  leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("Ri Enc Pos", rightFrontEncoder.getPosition());

    /**
     * 
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Le Enc Vel",  leftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Ri Enc Vel", rightFrontEncoder.getVelocity());
  
  }
  
  @Override
  public void disabledInit() {
  }
  
  @Override
  public void disabledPeriodic() {
  }
  
  @Override
  public void testInit() {
    resetEncoders();
  }
  
  @Override
  public void testPeriodic() {
    getJoystickCoords();
    
    if(joy1.getRawButton(9)){
    
      int ii = 0;
      int minimumIi = 4;
      
      for(int i = 0; i < objectWasInView.length; i++){
        if(objectWasInView[i]) ii++;
      }
      if(!objectInView && ii > 0){
        objectSeenCount ++;
        objectInView = true;
      }
      else if(objectInView && ii < minimumIi){
        objectInView = false;
      }
      
      objectWasInView[llTurnFrameIndex % llTurnFrameWavelength] = (lla > 0.01);
      llTurnFrameIndex ++;
      
      // SmartDashboard.putNumber("objectSC"    , objectSeenCount      );
      // SmartDashboard.putNumber("objectSIi"   , ii                   );
      // SmartDashboard.putNumber("objectFIndex", llTurnFrameIndex     );
      // SmartDashboard.putNumber("objectFLen"  , llTurnFrameWavelength);
      
      turn = 0.9;
      
      // the number of encoder units per full revolution (of 360 deg)
      SmartDashboard.putNumber("enc per spin", leftFrontEncoder.getPosition() / (objectSeenCount / 2));
      
    }
    
    driveTheRobot();
  }
  
  @Override
  public void simulationInit() {
  }
  
  @Override
  public void simulationPeriodic() {
  }
}


