package frc.robot.subsystems.Shooter;

public class AlphaShooter extends Shooter {
   private static AlphaShooter instance; 

   public static AlphaShooter getInstance() {
        if (instance == null) {
            instance = new AlphaShooter();
        }

        return instance;
   }

   private AlphaShooter() { super(); } 
}
