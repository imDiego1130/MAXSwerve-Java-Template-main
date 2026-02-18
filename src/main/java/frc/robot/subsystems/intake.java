package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class intake {
    private final SparkMax m_roller1Spark;
    private final SparkMax m_rollerOHSpark;

    private final RelativeEncoder m_roller1Encoder;
    private final RelativeEncoder m_rollerOHEncoder;
       private final SparkClosedLoopController m_roller1ClosedLoopController;
         private final SparkClosedLoopController m_rollerOHClosedLoopController;

    public intake(int roller1_RE, int rollerOH_RE){ 
      
         m_roller1Spark = new SparkMax(roller1_RE, MotorType.kBrushless);
          m_rollerOHSpark = new SparkMax(rollerOH_RE, MotorType.kBrushless);

        m_roller1Encoder = m_roller1Spark.getEncoder();
        m_rollerOHEncoder = m_rollerOHSpark.getEncoder();

           m_roller1ClosedLoopController = m_roller1Spark.getClosedLoopController();
            m_rollerOHClosedLoopController = m_rollerOHSpark.getClosedLoopController();

            m_rollerOHEncoder.setPosition(0);
    }}

