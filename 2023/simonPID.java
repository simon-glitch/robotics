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
