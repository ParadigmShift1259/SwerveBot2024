#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class GoToPositionCommand: public frc2::CommandHelper<frc2::Command, GoToPositionCommand>
{
    public:
        explicit GoToPositionCommand(ISubsystemAccess& subsystemAccess, bool bIsBlue);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        DriveSubsystem&        m_driveSubsystem;
        VisionSubsystem&        m_visionSubsystem;
        double m_targetX;
        double m_targetY;
};
