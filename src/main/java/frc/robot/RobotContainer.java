package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveModuleTesting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.BallIntake;


public class RobotContainer {
    
    private final XboxController Player1Controller = new XboxController(0);
    private final XboxController Player2Controller = new XboxController(1);

    private final SwerveDriveSubsystem module1 = new SwerveDriveSubsystem();
    private final Elevator elevador = new Elevator(RobotConstants.ELEVATOR_ID);
    private final Hanger colgador = new Hanger(RobotConstants.HANGER_ID);
    private final Intake tomador = new Intake(RobotConstants.ARM_ID,RobotConstants.INTAKE_ID);
    private final BallIntake bola = new BallIntake(RobotConstants.MARCO_ID,RobotConstants.BALLINTAKE_ID);
    
    public RobotContainer() {
        module1.setDefaultCommand(new DriveWithJoysticks(module1, Player1Controller));
        
        configureButtonBindings();
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
    
    private void configureButtonBindings() {

      final JoystickButton button1Start = new JoystickButton(Player1Controller, 8);
      final JoystickButton button1Select = new JoystickButton(Player1Controller, 7);

      final JoystickButton button1A = new JoystickButton(Player1Controller, 1);
      final JoystickButton button1B = new JoystickButton(Player1Controller, 2);
      final JoystickButton button1X = new JoystickButton(Player1Controller, 3);
      final JoystickButton button1Y = new JoystickButton(Player1Controller, 4);

      final JoystickButton button2A = new JoystickButton(Player2Controller, 1);
      final JoystickButton button2B = new JoystickButton(Player2Controller, 2);
      final JoystickButton button2X = new JoystickButton(Player2Controller, 3);
      final JoystickButton button2Y = new JoystickButton(Player2Controller, 4);
      final JoystickButton button2BumperL = new JoystickButton(Player2Controller, 5);
      final JoystickButton button2BumperR = new JoystickButton(Player2Controller, 6);
      final JoystickButton button2Select = new JoystickButton(Player2Controller, 7);
      final JoystickButton button2Start = new JoystickButton(Player2Controller, 8);
//Funciones botones básicos Player1
        // Cambia entre field oriented y robot oriented usando el botón A 

        button1Start.onTrue(new InstantCommand(() -> module1.setFieldOrientedTrue()));
        button1Select.onTrue(new InstantCommand(() -> module1.setFieldOrientedFalse()));

        button1A.onTrue(new InstantCommand(() -> bola.marcoSube(),bola))
        .onFalse(new InstantCommand(() -> bola.marcoStop(),bola));

        button1B.onTrue(new InstantCommand(() -> bola.marcoBaja(),bola))
        .onFalse(new InstantCommand(() -> bola.marcoStop(),bola));

        button1X.onTrue(new InstantCommand(() -> bola.ballIntakeComer(),bola))
        .onFalse(new InstantCommand(() -> bola.ballIntakeStop(),bola));

        button1Y.onTrue(new InstantCommand(() -> bola.ballIntakeSacar(),bola))
        .onFalse(new InstantCommand(() -> bola.ballIntakeStop(),bola));


//Funciones botones básicos Player2
        button2A.onTrue(new InstantCommand(() -> elevador.elevatorSube(),elevador))
        .onFalse(new InstantCommand(() -> elevador.elevatorStop(),elevador));

        button2B.onTrue(new InstantCommand(() -> elevador.elevatorBaja(),elevador))
        .onFalse(new InstantCommand(() -> elevador.elevatorStop(),elevador));

        button2X.onTrue(new InstantCommand(() -> colgador.hangerSube(),colgador))
        .onFalse(new InstantCommand(() -> colgador.hangerStop(),colgador));

        button2Y.onTrue(new InstantCommand(() -> colgador.hangerBaja(),colgador))
        .onFalse(new InstantCommand(() -> colgador.hangerStop(),colgador));

        button2BumperL.onTrue(new InstantCommand(() -> tomador.armSube(),tomador))
        .onFalse(new InstantCommand(() -> tomador.armStop(),tomador));

        button2BumperR.onTrue(new InstantCommand(() -> tomador.armBaja(),tomador))
        .onFalse(new InstantCommand(() -> tomador.armStop(),tomador));

        button2Select.onTrue(new InstantCommand(() -> tomador.intakeComer(),tomador))
        .onFalse(new InstantCommand(() -> tomador.intakeStop(),tomador));

        button2Start.onTrue(new InstantCommand(() -> tomador.intakeSacar(),tomador))
        .onFalse(new InstantCommand(() -> tomador.intakeStop(),tomador)); 
    }
}


