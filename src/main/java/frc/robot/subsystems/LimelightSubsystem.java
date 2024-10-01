package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

// Completely rewritten from the previous version for clarity and simplicity

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightSubsystem extends SubsystemBase {
    // NetworkTables allow different parts of the robot to communicate with each other
    // This one will be connected to the Limelight (smart camera) and will allow the subsystem to read data from it
    NetworkTable limelightTable;

    // NetworkTableEntries store the location of a single piece of data in a NetworkTable
    NetworkTableEntry targetX, targetY, targetArea, targetTag;

    SwerveSubsystem swerve;
    XboxController controller;
    DoubleSupplier triggerSupplier = () -> controller.getRightTriggerAxis();

    Command driveCommand;
    Command backCommand;
    Command stopCommand;

    double speed = 0.25;
    double correction = 0.03;
    double rumbleDuration = 0.1;
    boolean seen = false;

    // Java doesn't supports structs like C, so this is how you would implement one
    public class LimelightData {
        double targetX;
        double targetY;
        double targetArea;
        long   targetTag; // -1 when no tag is in focus, the ID of the tag if else
    }

    public LimelightSubsystem(SwerveSubsystem swerve, XboxController driveController) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        targetX        = limelightTable.getEntry("tx");
        targetY        = limelightTable.getEntry("ty");
        targetArea     = limelightTable.getEntry("ta");
        targetTag      = limelightTable.getEntry("tid");

        this.swerve = swerve;
        this.controller = driveController;

        driveCommand = swerve.simDriveCommand(
            () -> triggerSupplier.getAsDouble() * speed,
            () -> triggerSupplier.getAsDouble() * ((targetX.getDouble(13.5) - 13.5) / (13.5 / speed)),
            () -> 0);  //targetX.getDouble(0) * correction * speed * triggerSupplier.getAsDouble());

        backCommand = swerve.simDriveCommand(
            () -> triggerSupplier.getAsDouble() * (-speed),
            () -> 0,
            () -> 0);

        stopCommand = swerve.simDriveCommand(
            () -> 0,
            () -> 0,
            () -> 0);
    }

    public LimelightData getData() {
        // Create a new LimeLight data type named ret, because this object will be returned
        LimelightData ret = new LimelightData();

        // Fill ret with the proper data, the parameter of get<Type> is the default value
        ret.targetX    = targetX.getDouble(0);
        ret.targetY    = targetY.getDouble(0);
        ret.targetArea = targetArea.getDouble(0);
        ret.targetTag  = targetTag.getInteger(0);

        return ret;
    }

    public class Stop extends Command {
        @Override
        public void execute() {
            driveCommand.cancel();
            backCommand.cancel();
            speed = 0;
            stopCommand.execute();
        }

        @Override
        public boolean isFinished() { return true; }
    }

    @Override
    public void periodic() {
        // Periodically, push the values from the Limelight to the SmartDashboard
        SmartDashboard.putNumber("[LIMELIGHT] TargetX ",    targetX.getDouble(0));
        SmartDashboard.putNumber("[LIMELIGHT] TargetY ",    targetY.getDouble(0));
        SmartDashboard.putNumber("[LIMELIGHT] TargetArea ", targetArea.getDouble(0));
        SmartDashboard.putNumber("[LIMELIGHT] TargetTag ",  targetTag.getInteger(0));

        if(targetTag.getInteger(-1) != -1) {
            if(!seen) {
                seen = true;
                controller.setRumble(RumbleType.kRightRumble, 1);
                Timer.delay(rumbleDuration);
                controller.setRumble(RumbleType.kRightRumble, 0);
                Timer.delay(rumbleDuration);
                controller.setRumble(RumbleType.kRightRumble, 1);
                Timer.delay(rumbleDuration);
                controller.setRumble(RumbleType.kRightRumble, 0);
            }
            switch((int)targetTag.getInteger(-1)) {
                case 1: if(!driveCommand.isScheduled()) driveCommand.schedule(); break;
                case 2: if(!backCommand.isScheduled())  backCommand.schedule();  break;
            }
        } else {
            driveCommand.cancel();
            backCommand.cancel();
            stopCommand.execute();
            seen = false;
            controller.setRumble(RumbleType.kRightRumble, 0);
        }

        SmartDashboard.putNumber("[LIMELIGHT] Speed ", speed);
        SmartDashboard.putNumber("[LIMELIGHT] Joystick Position ", controller.getRightTriggerAxis());
    }
}
