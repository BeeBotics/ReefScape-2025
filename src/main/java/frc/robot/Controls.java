package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

public class Controls {
    public static void configureControls(int port, Elevator elevator, Arm arm,
            Climber climber, CoralSim coralSim) {

        CommandXboxController controller = new CommandXboxController(port);

       

        controller.a().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, elevator, arm, coralSim));
        controller.x().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
        controller.b().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
        controller.y().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
   

        controller.povUp().whileTrue(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
        controller.povDown().whileTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));

        controller.povLeft().whileTrue(RobotCommands.prepareAlgaeL2RemoveCommand(elevator, arm));
        controller.povRight().whileTrue(RobotCommands.prepareAlgaeL3RemoveCommand(elevator, arm));
     

        climber.setDefaultCommand(Commands
                .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 4,
                                0.1)),
                        climber));

        controller.povRight().onTrue(GlobalStates.INITIALIZED.enableCommand());

    }
}
