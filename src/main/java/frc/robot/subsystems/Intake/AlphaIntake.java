package frc.robot.subsystems.Intake;

public class AlphaIntake extends Intake {
    private static AlphaIntake instance;

    public static AlphaIntake getInstance() {
        if (instance == null) {
            instance = new AlphaIntake();
        }

        return instance;
    }
    

    private AlphaIntake() {
        super();
    }
}
