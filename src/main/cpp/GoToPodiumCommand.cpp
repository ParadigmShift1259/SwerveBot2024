#include "GoToPodiumCommand.h"

GoToPodiumCommand::GoToPodiumCommand(ISubsystemAccess& subsystemAccess)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
{
    // AddRequirements(&subsystemAccess.GetVision());
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});
}

void GoToPodiumCommand::Initialize()
{

}

void GoToPodiumCommand::Execute()
{

}

bool GoToPodiumCommand::IsFinished()
{
    return true;
}

void GoToPodiumCommand::End(bool interrupted)
{

}
