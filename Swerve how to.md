# Swerve how to
## 1. Create project
## 2. Add vendor libraries
1. REVLib
   
   The REVLib library add support for the REV SparkMAX motor controller.
   
   Follow these [instructions to install REVLIB library](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#online-installation).

2. CTRE Phoenix

    The CTRE Phoenix library adds support for hardware created by CTRE, such as Talon FX (Falcon 500), Talon SRX, Victor SPX,
    CANdle, CANivore, and the CANCoder.
    
    Follow [these instructions to Phoenix library]().

3. SDS Swervelib

    Copy the file content of [SDS Swerve Library Config](https://raw.githubusercontent.com/nowireless/TurretSwerve/main/vendordeps/SdsSwerveLib.json) to vendordeps/SdsSwerveLib.json
    Then WPILIB -> Manage Vender Libraries -> Install new (Offline)
    
## 3. Constants
### 1. Add CANBus enum
Add a enumeration to define what CANBus a device is connected to. CTRE devices (such as the CANCoder) can be connected to
either the roboRIOs CANBus or the CANivore. SparkMAXes can be only connected to the roboRIO canbus.
```java
     ...truncated...
     public final class Constants {
         ...truncated...

        public enum CANBus {
            kRoboRIO("rio"),
            kCANivore("canivore");
    
            public final String busName;
    
            private CANBus(String busName) {
                this.busName = busName;
            }
        }
     }
```

### 2. Drive Constants
1. In `Constants.java` add a nested class to store our drive constants:
    ```java
    ...truncated...
    public final class Constants {
        ...truncated...
      
        public static class DriveConstants {
           //
           // Physical constants
           //
    
           //
           // IMU
           //
       
           //
           // Individual module configuration
           //
        }
    }
    ```

1. Add physical constants under the ` // Physical constants` comment.

    1. Maximum allowed drive voltage when voltage compenstation is enabled.
        ```java
        public static final double kDriveVoltageCompensation = 10;
        ```
    
    1. Size of the swerve module wheel.
        ```java
        // Size of the module wheel
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        ```
    
    1. Distance between centers of the front and rear wheels on robot.
        ```java
        // Distance between centers of the front and rear wheels on robot
        public static final double kWheelBaseLengthMeters = Units.inchesToMeters(21.5);
        ```
       
    1. Distance between centers of right and left wheels on robot.
        ```java
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidthMeters = Units.inchesToMeters(26);
        ```
       
    1. Drive wheel gear reduction.
        ```java
        public static final double kDriveGearReduction = SASModuleHelper.GearRatio.V2.getConfiguration().getDriveReduction();
        ```
    
    1. Determine the max drive (linear) velocity in meters per second
        ```java
        //  FreeSpeed Radians   1 Rotation                     kWheelDiameter Meters   Voltage Nominal   FreeSpeed * kGearReduction * kWheelDiameter Meters
        //  ----------------- * ----------- * kGearReduction * --------------------- * --------------- = --------------------------------------------------
        //  1 Second            2PI Radians                    1 Rotation              Voltage Max       2PI Second
        public static final double kMaxDriveVelocityMetersPerSecond = DCMotor.getNEO(1).freeSpeedRadPerSec / (2*Math.PI) * kDriveGearReduction * kWheelDiameterMeters * (kDriveVoltageCompensation/12.0);
        ```
       
        > Luckily for use the WPILib has a class containing information about about motors, such as their free speed.  
    
    1. Determine the max angular velocity
       ```java
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         *
         * Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
         */
        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxDriveVelocityMetersPerSecond /
           Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseLengthMeters / 2.0);
       ```
       
    1. Setup `SwerveDriveKinematics`:
        ```java
        public static final SwerveDriveKinematics kDriveKinematics =
                   new SwerveDriveKinematics(
                       new Translation2d(kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
                       new Translation2d(kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2),
                       new Translation2d(-kWheelBaseLengthMeters / 2, kTrackWidthMeters / 2),
                       new Translation2d(-kWheelBaseLengthMeters / 2, -kTrackWidthMeters / 2));
        ```

1. Add IMU constants under the ` // IMU` comment.
    1. CAN ID of the Pigeon IMU
        ```java
        public static final int kPigeonID = 0;
        ```
    
    1. Specify the CAN Bus the Pigeon is connected to.
        ```java
        public static final CANBus kPigeonCANBus = CANBus.kCANivore;
        ```
    
1. Add individual module configuration under the `// Individual module configuration` comment
    1. Add the CAN ID for the drive motor SparkMAX.
        ```java
        public static final int kFrontLeftMotorDriveID = 10;
        ```
    
    1. Add the CAN ID for the steer motor SparkMAX.
        ```java
        public static final int kFrontLeftMotorSteerID = 11;
        ```
       
    1. Add the CAN ID for the steer CANcoder.
        ```java
        public static final int kFrontLeftEncoderID = 11;
        ```
    
    1. Specify the offset for the swerve module, for right now an offset of 0 will be fine and configured later.
        ```java
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromDegrees(-64);
        ```
    
    1. Repeat the above steps for each module. Replace the `kFrontLeft` prefix with `kRearLeft`, `kFrontRight`,
        `kRearRight`. 
    
      **Important** ensure the following:
      - All motors CAN IDs are unique. The easiest scheme would be to start at 10, and count up.
      - All CAN Coder IDs are unique (the IDs can overlap with motors IDs, as they do not overlap). The CANCoder ID can 
        be set to the ID of the steer motor ID.

## 4. Drivetain Subsystem
1.  Create the Drivetrain.

    ```java
    package frc.robot.subsystems;
    
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
        
    public class Drivetrain extends SubsystemBase {
        // Helpers
    
        // Hardware
    
        public Drivetrain() {
            //
            // IMU setup
            //
            // TODO       

            //
            // Swerve setup
            //
            // TODO       
        }
    
        @Override
        public void periodic() {    
        }
    }
    ```

1.  Add static imports after the list imports near the top of the file (after line 3), so we can easily access our constants.

    > This will allow us to reference the `DriveConstants` from `Constants.java` using 
    > `DriveConstants.kFooBar` instead of using a long name like `Constants.DriveConstants.kFooBar`.

    ```java
    
    import static frc.robot.Constants.*;
    
    ```

1. Time to add our first piece of hardware the Internal Measurement Unit (IMU). A IMU is a neat sensor
   that can measure the orientation of the robot. This is really useful at it can measure the robot's heading (yaw) angle, and
   enables some cool features of the robot:
    - **Field Centric control**: Instead of controlling the movements of the robot from the positive of the robot 
      (moving the joystick forward, it will move the robot forward). When in Field Centric control, when the joysitck is moved
      the robot will move "north" regardless of the robots heading ("north" is defined as the direction the driver is facing). 
    - **Odometry**: The IMU is one of the sensors needed for odometry. Odometry is process of using sensors on the robot
      to estimate the position of the robot on the field. This will enable some fancy autos.

    1. Create a member variable for the IMU under the `// Hardware` comment.
       ```java
       private final Pigeon2 m_imu;
       ```

    1. In the Drivetrain constructor lets create a new Pigeon IMU.
        ```java
            //
            // IMU setup
            //
            m_imu = new Pigeon2(DriveConstants.kPigeonID, DriveConstants.kPigeonCANBus.busName);
        ```
       
    1. Add the `getHeading()` method to retrieve the current IMU heading as a [Rotation2d object](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html?highlight=rotation2d#rotation).
       The Rotation2d object has a few advantages over just storing the angle in a `double`:
       - Easily convert from both degrees (human friendly) and radians (computer friendly).
       - Easily [perform transforms operations on angles](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/transformations.html#rotation2d)
       - Automatically normalizes angles between -180 and 180. So a large angle like 405 degrees becomes 45 degrees.  
   
       ```java
          public Rotation2d getHeading() {
               return Rotation2d.fromDegrees(m_imu.getYaw());
          }
       ```
       
       > The `Rotation2d.fromDegrees(angleDegress)` static method can an angle in degrees into a Rotation2d object.
       > 
       > If you have radians, then `new Rotation2d(angleRadians)` can be used.

    1. Add the `zeroHeading()` method. This will set the current yaw/heading angle of the IMU to 0. This is useful if we
       when the robot is out on the field, and we want to set the current direction teh robot is facing as "north" by making the 
       current angle now 0.
   
       ```java
       public void zeroHeading() {
           m_imu.setYaw(0);
       }
       ```
   
1. To help ease troubleshooting we can put information about the IMU to Shuffleboard. Shuffleboard has some cool functionality
    where we can define the layout of the dashboard in our robot code. The following should be added in the Drivetrain constuctor
    after the Pigeon is created.

    1. First we need to get a reference to the `Drivetrain` tab in Shuffleboard.
       
        ```java
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ```

    1. Next need to create a layout to group together data from the IMU. This will make a 2 by 2 square located in column 8 and row 0.
        ````java    
        var imuShuffleboard = tab.getLayout("IMU", BuiltInLayouts.kList)
            .withSize(2,2)
            .withPosition(8, 0);
        ```   

    1. Probably the most important piece of information about the IMU to send is the IMU heading.
        ```java
        imuShuffleboard.addNumber("Heading", ()->getHeading().getDegrees());
        ```

    1. The IMU also has a temperature sensor. The [`getTemp()` method](https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/sensors/BasePigeon.html#getTemp()) reports the temperature in celsius, so it is converted to fahrenheit.
        ```java
        imuShuffleboard.addNumber("Temperature", ()-> m_imu.getTemp() * 9.0/5.0+32.0);
        ```

    > Shuffleboard is the dashboard that we use on the driver station. For more information about it
    > [check out the WPILib docs](https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html?highlight=shuffleboard).

1. Now it is time to add our swerve modules!

    1. Add the following member variables under the `// Hardware` comment for each of our swerve modules.
       ```java
       private final SwerveModule m_frontLeft;
       private final SwerveModule m_rearLeft;
       private final SwerveModule m_frontRight;
       private final SwerveModule m_rearRight;
       ```

    1. In the constructor under `// Swerve Modules` comment lets add some common module configuration. 
       We want to configure voltage compensation for our motors using the value `DriveConstants.kDriveVoltageCompensation` 
       we defined in our `Constants.java` file. Voltage compensation will limit the max voltage the Spark MAX will send 
       to the motor (in our case 10 volts). This is useful as it will help the robot's motor perform the same deal with 
       varying battery voltage levels. Motors will move the same speed if the battery is at 13 volts or if the battery is
       at 10 volts.  
       
       > TODO move DriveConstants into DriveConstants  
    
      ```java
      Mk4ModuleConfiguration moduleConfiguration = new Mk4ModuleConfiguration();
      moduleConfiguration.setNominalVoltage(DriveConstants.kDriveVoltageCompensation);
      ```

    1. Create the swerve module object. 
    
        The swerve module we have has the following characteristics:
        - SDS Mk4 Module
        - L2 Gearing
        - NEO Drive motor
        - NEO Steer motor
        - CANCoder encoder for module angle.
    
        The following information can be provided to the `Mk4SwerveModuleHelper.createNeo` method:
        1. Shuffleboard layout to contain debugging information about the module such as the modules
           current angle and velocity.
        1. The module configuration we created in the previous step.
        1. The gear ration of the module (there are 4 gearings available).
        1. CAN ID for the NEO drive motor
        1. CAN ID for the NEO steer motor
        1. CAN ID for the CANCoder
        1. The module offset in radians.
    
        ```$xslt
        m_frontLeft = Mk4SwerveModuleHelper.createNeo(
            // Push module information to shuffle boards
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 6)
                .withPosition(0, 0), // Column, Row
            moduleConfiguration,
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.kFrontLeftMotorDriveID,
            DriveConstants.kFrontLeftMotorSteerID,
            DriveConstants.kFrontLeftEncoderID,
            DriveConstants.kFrontLeftOffset.getRadians()
        );
       ```
    
    1. Repeat the previous step with the  `m_rearLeft`, `m_frontRight`, and `m_rearRight` modules.
    
        | Module         | Layout Name          | Column | Row |
        | -------------- | -------------------- | ------ | --- |
        | `m_frontLeft`  | `Front Left Module`  | 0      | 0   |
        | `m_rearLeft`   | `Back Left Module`   | 2      | 0   |
        | `m_frontRight` | `Front Right Module` | 4      | 0   |
        | `m_rearRight`  | `Back Right Module`  | 5      | 0   |

1. Add the `setModuleStates()` method. A `SwerveModuleState` object contains information about the velocity and angle of
   a singular module of a swerve drive. The `setModuleStates()` method is given an array of desired module states (one 
   for each module).
   
   The overall flow of this method is the following:
   1. Ensure all module states have achievable values (ie the speeds do not exceed the maximums defined in 
      SwerveDriveKinematics in `Constants.java`) using `SwerveDriveKinematics.desaturateWheelSpeeds`. If any module is 
      going to fast all of the speeds will be scaled down to slow the modules down.
   1. **For each** module optimize the desired swerve module state. By comparing the current angle with 
      the desired angle using the `SwerveModuleState.optimize` method can determine if inverting the direction of the motor
      will create a shorter turn angle for the module. This should prevent the swerve modules from every turning more then 
      90 degrwss.  
   1. **For each** module set the desired speed and angle of the swerve module. 
        1. The desired speed in the SwerveModuleState is in meters per second, but `set` on the module requires a voltage.
           Since we know the max speed of the robot can calculate the percentage/ratio of how fast we want to the robot to move,
           and then scale it by the max voltage the motor can go.
        1. The desired module angle in radians.  
   
      > **Note** the order of the `desiredStates` array matches the order of the modules provided to 
      >SwerveDriveKinematics in `Constants.java`.
   
   ```java
    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
       // Ensure all module states have achievable values
       SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxDriveVelocityMetersPerSecond);
   
       // Optimize swerve module states. Prevent the swerve modules from moving farther then 90 degrees. If the direction
       // of the motor can be inverted
       desiredStates[0] = SwerveModuleState.optimize(desiredStates[0], new Rotation2d(m_frontLeft.getSteerAngle()));
       desiredStates[1] = SwerveModuleState.optimize(desiredStates[1], new Rotation2d(m_frontRight.getSteerAngle()));
       desiredStates[2] = SwerveModuleState.optimize(desiredStates[2], new Rotation2d(m_rearLeft.getSteerAngle()));
       desiredStates[3] = SwerveModuleState.optimize(desiredStates[3], new Rotation2d(m_rearRight.getSteerAngle()));
   
       // Send it!
       m_frontLeft.set(
           desiredStates[0].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * DriveConstants.kDriveVoltageCompensation,
           desiredStates[0].angle.getRadians()
       );
   
       m_frontRight.set(
           desiredStates[1].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * DriveConstants.kDriveVoltageCompensation,
           desiredStates[1].angle.getRadians()
       );
   
       m_rearLeft.set(
           desiredStates[2].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * DriveConstants.kDriveVoltageCompensation,
           desiredStates[2].angle.getRadians()
       );
   
       m_rearRight.set(
           desiredStates[3].speedMetersPerSecond / DriveConstants.kMaxDriveVelocityMetersPerSecond * DriveConstants.kDriveVoltageCompensation,
           desiredStates[3].angle.getRadians()
       );
   }
   ```
   
   > For more information about:
   > - `SwerveModuleState`: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html?highlight=SwerveModuleState#the-swerve-module-state-class

1. Add the `drive()` method. Method to drive the robot using joystick information. 

    The following information is supplied to drive method:
    1. `xSpeed`: Speed of the robot in the x direction (forward) in meters per second. Positive values move forward.
    1. `ySpeed`: Speed of the robot in the y direction (sideways) in meters per second. Positive values move left.
    1. `rot`: Angular rate of the robot in radians per second. Positive values rotate counter-clockwise.
    1. `fieldRelative`: Whether the provided x and y speeds are relative to the field.
    
    If field relative/centric control is desired, then the `ChassisSpeeds.fromFieldRelativeSpeeds` method will be used to
    rotate the field centric x, y speeds from the field reference frame to the robot's point of view by using the current
    heading reported by the IMU.   

    The `ChassisSpeeds` object contains how fast the robot overall should move forwards, side to side, and rotate, but
    it does not contain the individual swerve module states. Depending on the desired movement all of the swerve modules
    will have a unique velocity and angle.
    
    The `DriveConstants.kDriveKinematics.toSwerveModuleStates` method can convert `ChassisSpeeds` of the overall robot 
    movement, and determine the individual module state.

    ```java
      /**
       * Method to drive the robot using joystick info. All speeds are in meters per second.
       *
       * For more information, about coordinate systems see:
       * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
       *
       * @param xSpeed Speed of the robot in the x direction (forward) in meters per second. Positive values move forward.
       * @param ySpeed Speed of the robot in the y direction (sideways) in meters per second. Positive values move left
       * @param rot Angular rate of the robot in radians per second. Positive values rotate counter-clockwise.
       * @param fieldRelative Whether the provided x and y speeds are relative to the field.
       */
      public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;
    
        if (fieldRelative) {
          // Field centric control
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        } else {
          // Robot centric control
          chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
    
        // Send it!
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
      }
    ```
   
   > For more information see: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#converting-chassis-speeds-to-module-states  

1. Add the `stop()` method. The `stop()` convenience method simply calls `drive()` with values to ensure the drive stops.

    ```java
    /**
     * Stop the drive train
     */
    public void stop() {
        drive(0, 0, 0, false);
    }
    ```


​    
## 5. DriveWithController Command

## N. Bring up
### 1. Configure radio
TODO
### 2. Update firmware
1. Update/verify roboRIO image using TODO.
1. Update firmware on SparkMAXes using TODO. 
1. Update firmware on CANCoders using TODO.
### 3. Configure CAN IDs to make code
TODO 

### 4. Determine module offset
TODO



## 5. Advanced
### 1. Odometry configuration
1. Add odometry functionality to the drive train. 
    > For more information see: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html?highlight=Odometry

    TODO

### 2. Auto using the Holonomic Drive Controller
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html

Constants

### 3. Prevent modules from turning unnecessarily

### 4. Calibrate Pigeon IMU: https://docs.ctre-phoenix.com/en/stable/ch11_BringUpPigeon.html
TODO 

