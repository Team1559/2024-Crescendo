// package frc.robot.subsystems.climber;

// import static frc.robot.constants.AbstractConstants.CONSTANTS;

// import org.littletonrobotics.junction.AutoLog;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climber extends SubsystemBase {

// @AutoLog
// static class ClimberInputs {

// }

// private final CANSparkMax motorL = new CANSparkMax(99, MotorType.kBrushless);
// private final CANSparkMax motorR = new CANSparkMax(99, MotorType.kBrushless);

// private final PIDController controller = new
// PIDController(CONSTANTS.getAimerPid().P, CONSTANTS.getAimerPid().I,
// CONSTANTS.getAimerPid().D);

// // private final ClimberInputsAutoLogged inputs = new
// ClimberInputsAutoLogged();

// /**
// * Create a new subsystem for two motors controlled by CANspark Controller
// **/
// public Climber() {
// motorL.setInverted(false);
// motorR.setInverted(true);
// motorL.setIdleMode(IdleMode.kBrake);
// motorR.setIdleMode(IdleMode.kBrake);
// motorL.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
// motorR.setSmartCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentLimit());
// motorL.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
// motorL.setSecondaryCurrentLimit(CONSTANTS.getNeo550BrushlessCurrentSecondaryLimit());
// }

// // @Override
// // public void periodic() {
// // // Log inputs
// // updateInputs();
// // Logger.processInputs("Shooter/Aimer", inputs);

// // motorL.set
// // }

// private void updateInputs() {
// }
// }
