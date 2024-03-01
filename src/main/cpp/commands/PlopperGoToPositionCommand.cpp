#include "commands/PlopperGoToPositionCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const double c_targetPodiumX = (2.896_m - 14.75_in).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.1;

const double c_targetSpeakerX = 1.26;
const double c_targetSpeakerY = 5.35;


PlopperGoToPositionCommand::PlopperGoToPositionCommand(ISubsystemAccess& subsystemAccess)
    : m_ampSubsystem(subsystemAccess.GetAmp())
{

    frc::SmartDashboard::PutNumber("Plopper Position", 0.0);
    AddRequirements(frc2::Requirements{&subsystemAccess.GetAmp()});
}

void PlopperGoToPositionCommand::Initialize()
{

}

void PlopperGoToPositionCommand::Execute()
{
    m_ampSubsystem.GoToPosition(frc::SmartDashboard::GetNumber("Plopper Position", 0.0));
}

bool PlopperGoToPositionCommand::IsFinished()
{
    return true;
}

void PlopperGoToPositionCommand::End(bool interrupted)
{
}