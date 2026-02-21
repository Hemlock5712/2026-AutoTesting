package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.cortex.math.geometry.ExtTranslation;
import frc.cortex.util.FieldInfo;

public class FieldConstants {
    public static final ExtTranslation HUB_POSITION = new ExtTranslation(Meters.of(4.621), FieldInfo.width().div(2));
    public static final Distance HUB_HEIGHT = Meters.of(1.828);
}