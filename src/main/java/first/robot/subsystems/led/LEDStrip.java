// package first.robot.subsystems.led;

// import org.wpilib.hardware.led.AddressableLED;
// import org.wpilib.hardware.led.AddressableLEDBuffer;
// import org.wpilib.hardware.led.LEDPattern;
// import org.wpilib.util.Color;

// import first.robot.Constants.LEDConstants;

// import org.wpilib.command2.Command;
// import org.wpilib.command2.RunCommand;
// import org.wpilib.command2.SubsystemBase;

// import java.util.function.BooleanSupplier;

// public class LEDStrip extends SubsystemBase {
//     private AddressableLED led;
//     private AddressableLEDBuffer ledBuffer;

//     private static final LEDStrip LEDSTRIP = new LEDStrip();

//     public static LEDStrip getInstance() {
//         return LEDSTRIP;
//     }

//     public LEDStrip() {
//         led = new AddressableLED(LEDConstants.PORT);
//         ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LEDS);
//         led.setLength(ledBuffer.getLength());
//         led.setData(ledBuffer);
//         led.start();

//         // can this be run periodically?
//         setGradient(Color.kGreen, Color.kGold);
//     }

//     public void periodic() {
//         // if (DriverStation.isDisabled()) {
//         //     setGradient(Color.kLightGreen, Color.kLightYellow);
//         // }
//         led.setData(ledBuffer);
//     }

//     public Command defaultCommand(BooleanSupplier isCoralSupplier) {
//         return new RunCommand(() -> setCoralOrAlgae(isCoralSupplier), this);
//     }

//     public Command aligning(BooleanSupplier isAligned) {
//         return new RunCommand(() -> setBreathing(Color.kViolet), this)
//                 .until(isAligned)
//                 .andThen(success());
//     }

//     public Command success() {
//         return new RunCommand(() -> setBlinking(Color.kGreen), this).withTimeout(LEDConstants.BLINKING_DURATION);
//     }

//     private void setCoralOrAlgae(BooleanSupplier isCoralSupplier) {
//         setSolid(isCoralSupplier.getAsBoolean() ? Color.kCoral : Color.kMediumAquamarine);
//     }

//     private void setSolid(Color color) {
//         LEDPattern solidPattern = LEDPattern.solid(color);
//         solidPattern.applyTo(ledBuffer);
//         led.setData(ledBuffer);
//     }

//     private void setGradient(Color... colors) {
//         LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.CONTINUOUS, colors)
//                 .scrollAtRelativeSpeed(LEDConstants.SCROLL_FREQ);
//         gradient.applyTo(ledBuffer);
//         led.setData(ledBuffer);
//     }

//     private void setBlinking(Color color) {
//         LEDPattern blinkPattern = LEDPattern.solid(color)
//                 .blink(LEDConstants.BLINK_PERIOD.div(2.0))
//                 .atBrightness(LEDConstants.BLINK_BRIGHTNESS);
//         blinkPattern.applyTo(ledBuffer);
//         led.setData(ledBuffer);
//     }

//     private void setBreathing(Color color) {
//         LEDPattern breathePattern = LEDPattern.solid(color).breathe(LEDConstants.BREATHE_PERIOD);
//         breathePattern.applyTo(ledBuffer);
//         led.setData(ledBuffer);
//     }
// }
