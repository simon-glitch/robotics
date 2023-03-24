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
