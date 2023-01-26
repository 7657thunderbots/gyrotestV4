package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Robot extends TimedRobot {
  PWMVictorSPX m_left, m_right;
  Joystick m_controller;
  ADIS16470_IMU m_gyro;
  int timer;
  SendableChooser<Double> m_chooser;
  

  //mag shooter
	private CANSparkMax intake;
	private CANSparkMax index;
	private CANSparkMax wormdrive;
	private CANSparkMax uptake1;
	private CANSparkMax uptake2;
	private CANSparkMax climber;
	private CANSparkMax climber2;
	
  //tank drive vars
	private DifferentialDrive tankDrive;
	private WPI_TalonFX leftParent;
	private WPI_TalonFX leftChild;
	private WPI_TalonFX rightParent;
	private WPI_TalonFX rightChild;

  private double speedMult;

  private Joystick left;
	private Joystick right;
	private XboxController controller2;

  private final Timer m_timer = new Timer();
  private final Timer m_timer2 = new Timer();
  private static boolean OpenFlag = false;
	private double wormStart = 0.0;

  static final double WORMSPEED=0.25;
	static final double WORMTIME=6.7;

  private boolean autolevelingage = false;
  private boolean chargelevel = false;
  
  private double turnerror =0.0;
  private double directionL =0.0;
  private double directionR =0.0;
  private double setpoint = 0.0;
  private double anglelast=0.0;
  private double newangle=0.0;
  private final PowerDistribution m_pdp = new PowerDistribution();
 
  @Override
  public void robotInit() {
    m_right = new PWMVictorSPX(0);
    m_left = new PWMVictorSPX(1);
    m_controller = new Joystick(0);
    m_gyro = new ADIS16470_IMU();
    m_chooser = new SendableChooser<>();

    m_right.setInverted(true);
    SmartDashboard.putData(m_chooser);

		speedMult = .5;

    DataLogManager.start();
    //cams
		CameraServer.startAutomaticCapture(0);
		CameraServer.startAutomaticCapture(1);
    
    //tankdrive
    leftParent = new WPI_TalonFX(4);
		leftChild = new WPI_TalonFX(5);
		leftParent.setInverted(true);
		leftChild.follow(leftParent);
		leftChild.setInverted(true);
		rightParent = new WPI_TalonFX(3);
		rightChild = new WPI_TalonFX(2);
		rightChild.follow(rightParent);
		tankDrive = new DifferentialDrive(rightParent, leftParent);

    left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
		
    leftParent = new WPI_TalonFX(4);
		leftChild = new WPI_TalonFX(5);
		leftParent.setInverted(true);
		leftChild.follow(leftParent);
		leftChild.setInverted(true);
		rightParent = new WPI_TalonFX(3);
		rightChild = new WPI_TalonFX(2);
		rightChild.follow(rightParent);
		tankDrive = new DifferentialDrive(rightParent, leftParent);

    

    //driveencoders
		rightParent.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		rightParent.setSelectedSensorPosition(0);
		leftParent.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		leftParent.setSelectedSensorPosition(0);

    // MAKE SURE GREEN CONTROLLER IS 0 IN DRIVER STATION!!!!!!!!!
		left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
	  }
  

  
  @Override
  public void autonomousInit() {
    m_gyro.reset();
    m_timer.reset();
		m_timer.start();
  m_timer2.reset();
  m_timer2.start();
  }


  

  @Override
  public void autonomousPeriodic() {
    
    if (m_timer.get() < 5 && m_gyro.getYComplementaryAngle()>-15)
    { directionL=.45;
    directionR=.45;
    }
   else if (m_timer.get()>=3)
   {
    setpoint = m_gyro.getYComplementaryAngle()*-.0275 - (newangle - anglelast)*.02;
    if (setpoint > 0.55){
      setpoint=.55;}
      if (setpoint<-.55){
        setpoint=-.55;}
      
    directionL=setpoint;
    directionR=setpoint;
   

        SmartDashboard.putNumber("robotangle", m_gyro.getYComplementaryAngle());
        SmartDashboard.putNumber("turnangle", m_gyro.getAngle());
        
    }
    if (m_gyro.getAngle()>3){
      turnerror = .1;}
      else if (m_gyro.getAngle()<3 && m_gyro.getAngle() >-3){
        turnerror =0;
      }
      else if (m_gyro.getAngle()<-3){
        turnerror =-.1;}
    tankDrive.tankDrive (turnerror+directionL,-turnerror+directionR);

    SmartDashboard.putData("PDP", m_pdp);
    SmartDashboard.putNumber("tilt angle", m_gyro.getYComplementaryAngle());
    SmartDashboard.putNumber("voltage",m_pdp.getVoltage());
   SmartDashboard.putNumber("PDP current", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total energy", m_pdp.getTotalEnergy());
    SmartDashboard.putNumber("Total power", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(9));
    SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(8));
    SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(10));
    SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(11));
    SmartDashboard.putNumber("turn angle",m_gyro.getAngle());
    SmartDashboard.putNumber("Constant speed left",directionL);
    SmartDashboard.putNumber("constant speed right", directionR);
    SmartDashboard.putNumber("error adjustment direction", turnerror);
    SmartDashboard.putNumber("X acceleration",m_gyro.getAccelX());
    SmartDashboard.putNumber("Y acceleration",m_gyro.getAccelY());
    SmartDashboard.putNumber("Z acceleration",m_gyro.getAccelZ());
    SmartDashboard.putNumber("Gyro Rate",m_gyro.getRate());
    SmartDashboard.putNumber("filtered x acceleration Angle",m_gyro.getXFilteredAccelAngle() );
    SmartDashboard.putNumber("filtered y acceleration Angle",m_gyro.getYFilteredAccelAngle() );
    SmartDashboard.putNumber("X Complimentary angle",m_gyro.getXComplementaryAngle() );
  SmartDashboard.putNumber("setpoint", setpoint);
  SmartDashboard.putNumber("newangle", newangle);
  SmartDashboard.putNumber("oldangle", anglelast);
  SmartDashboard.putNumber("timer", m_timer.get());
    anglelast=m_gyro.getYComplementaryAngle();
  }


      
    
    
      


  
    

  @Override
  public void teleopPeriodic() {
    m_right.set(-m_controller.getRawAxis(3));
    m_left.set(-m_controller.getRawAxis(1));
    System.out.println(m_gyro.getYComplementaryAngle());
    if (autolevelingage){
			SmartDashboard.putString("autolevelingage", "true");
		}else {
			SmartDashboard.putString("autolevelingage", "false");
	    }
      if (chargelevel){
        SmartDashboard.putString("chargelevel", "true");
      }else {
        SmartDashboard.putString("chargelevel", "false");
        }
            SmartDashboard.putData("PDP", m_pdp);
            SmartDashboard.putNumber("tilt angle", m_gyro.getYComplementaryAngle());
            SmartDashboard.putNumber("voltage",m_pdp.getVoltage());
           SmartDashboard.putNumber("PDP current", m_pdp.getTotalCurrent());
            SmartDashboard.putNumber("Total energy", m_pdp.getTotalEnergy());
            SmartDashboard.putNumber("Total power", m_pdp.getTotalPower());
            SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(9));
            SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(8));
            SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(10));
            SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(11));
            SmartDashboard.putNumber("turn angle",m_gyro.getAngle());
            SmartDashboard.putNumber("Constant speed left",directionL);
            SmartDashboard.putNumber("constant speed right", directionR);
            SmartDashboard.putNumber("error adjustment direction", turnerror);
            SmartDashboard.putNumber("X acceleration",m_gyro.getAccelX());
            SmartDashboard.putNumber("Y acceleration",m_gyro.getAccelY());
            SmartDashboard.putNumber("Z acceleration",m_gyro.getAccelZ());
            SmartDashboard.putNumber("Gyro Rate",m_gyro.getRate());
            SmartDashboard.putNumber("filtered x acceleration Angle",m_gyro.getXFilteredAccelAngle() );
            SmartDashboard.putNumber("filtered y acceleration Angle",m_gyro.getYFilteredAccelAngle() );
            SmartDashboard.putNumber("X Complimentary angle",m_gyro.getXComplementaryAngle() );
            SmartDashboard.putNumber("setpoint", setpoint);
            SmartDashboard.putNumber("newangle", newangle);
            SmartDashboard.putNumber("oldangle", anglelast);
          }
          
            
        
            
            
      

        
  
        }
