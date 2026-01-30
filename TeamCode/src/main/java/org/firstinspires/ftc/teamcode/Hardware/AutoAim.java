package org.firstinspires.ftc.teamcode.Hardware;


public class AutoAim {
    //auto aim for turret

    public Limey limey = new Limey();
    public Turret turret = new Turret();

    private final double OFFSET = 0.2032; // 8 inches in meters

    public void TurretAutonAim(Limey limey,Turret turret) {
        this.limey = limey;
        this.turret = turret;
    }

    public void update() {
        if(limey.getTagID() == -1){
            return;
        }

       // var result = limey.getResult();
      //  if ( result == null || result.getFiducialResults().isEmpty()) return;

       //     Pose3D`pose = result.Fid

    }




}
