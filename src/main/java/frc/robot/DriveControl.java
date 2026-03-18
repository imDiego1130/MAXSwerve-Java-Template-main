package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class DriveControl {
    private XboxController driver;
    private XboxController operator;

    public  DriveControl(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    public Rotation2d getJoyStickAngle(){
        double x = MathUtil.applyDeadband(operator.getRightX(), 0.1);
        double y = -MathUtil.applyDeadband(operator.getRightY(), 0.1);

        if (Math.hypot(x, y) < 0.15){
            return null;
        }

        return Rotation2d.fromRadians((Math.atan2(x, y)));
    }

    public double getForward(){
        return -0.9*MathUtil.applyDeadband(driver.getLeftY(), 0.15);
    }

    public double getStrafe(){
        return -0.9*MathUtil.applyDeadband(driver.getLeftX(), 0.15);
    }

    public double getRotate(){
        return -0.7*MathUtil.applyDeadband(driver.getRightX(), 0.15);
    }
}
