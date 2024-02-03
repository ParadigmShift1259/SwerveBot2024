
#include "subsystems/IntakeSubsystem.h"

//#include <frc/SmartDashBoard/SmartDashboard.h>
//#include <frc/shuffleboard/Shuffleboard.h>

using namespace frc;

IntakeSubsystem::IntakeSubsystem() 
    //: m_motor(kIntakeCANID)
{
    // m_motor.SetNeutralMode(NeutralMode::Coast);
    // m_motor.SetInverted(true);
}

void IntakeSubsystem::Periodic()
{
    // SmartDashboard::PutNumber("D_I_Motor", m_motor.Get());
}

void IntakeSubsystem::Set(double speed)
{
    // m_motor.Set(ControlMode::PercentOutput, speed);//kMotorReverseConstant);
}

void IntakeSubsystem::ExtendIntake()
{

}

void IntakeSubsystem::RetractIntake()
{
    
}
