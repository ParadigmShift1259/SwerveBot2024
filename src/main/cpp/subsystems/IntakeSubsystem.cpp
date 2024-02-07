
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeCANID)
{
    m_motor.SetNeutralMode(NeutralMode::Coast);
}

void IntakeSubsystem::Periodic()
{
    SmartDashboard::PutNumber("D_I_Motor", m_motor.GetMotorOutputVoltage());
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{

}

void IntakeSubsystem::RetractIntake()
{
    
}
