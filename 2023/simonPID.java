  private class PID{
    public double kP = 0.05;
    public double kI = 0.015;
    public double kD = 0.001;
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
    public double output;
    
    public boolean avoidJerk;
    
    public String nickname;
    
    /**
     * set the constants on this PID controller
     * @param p: value of kP (proportional constant) - should be positive
     * @param i: value of kI (integral     constant) - should be small
     * @param d: value of kD (derivative   constant) - should be negative
     */
    public void setup(double p, double i, double d){
      kP = p;
      kI = i;
      kD = d;
    }
    
    public void evolvePID(
      double expSign,
      boolean doEvolveP, boolean doEvolveI, boolean doEvolveD
    ){
      double mult = Math.pow(kexp, expSign * dt);
      SmartDashboard.putNumber(nickname + "expSign", expSign);
      SmartDashboard.putNumber(nickname + "mult"   , mult   );
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
     * set a new setPoint for this PID controller; beware encoders! they need to be zeroes as well
     * @param newPoint is the new setPoint
     */
    public void controlInit(double newPoint){
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
    
    public void controlPeriodic(){
      
      // get sensor position
      currPos  = rightFrontEncoder.getPosition();
      currPos -= kOffset;
      currPos *= kFactor;
      
      // calculations
      lastError = error;
      error = setPoint - currPos;
      
      if (Math.abs(error) < kiLimit) {
        errorSum += error * dt;
      }
      errorSum *= 0.9;
      
      errorRate = (lastError - error) / dt;
      if(avoidJerk){
        avoidJerk = false;
        errorRate = 0;
      }
      
      // actual PID formula
      double driveSpeed = kP * error + kI * errorSum + kD * errorRate;
      
      power = driveSpeed;
      turn = 0;
      
    }
    
    /**
     * Beware encoders! they need to be zeroes as well
     */
    public void zero(){
      controlInit(0);
      errorSum = 0;
      prevTimestamp = 0;
      lastError = 0;
    }
    
    public void log(){
      SmartDashboard.putNumber(nickname +" P.", kP);
      SmartDashboard.putNumber(nickname +" I.", kI);
      SmartDashboard.putNumber(nickname +" D.", kD);
      
      SmartDashboard.putNumber(nickname +" c pos" , currPos );
      SmartDashboard.putNumber(nickname +" set p" , setPoint);
      SmartDashboard.putNumber(nickname +" offset", kOffset );
      SmartDashboard.putNumber(nickname +" conv f", kFactor );
      
      SmartDashboard.putNumber(nickname +" err"    , error    );
      SmartDashboard.putNumber(nickname +" errSum" , errorSum );
      SmartDashboard.putNumber(nickname +" errRate", errorRate);
    }
  }
