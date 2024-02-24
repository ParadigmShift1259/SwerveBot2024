#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class PlopperGoToPositionCommand: public frc2::CommandHelper<frc2::Command, PlopperGoToPositionCommand>
{
    public:
        explicit PlopperGoToPositionCommand(ISubsystemAccess& subsystemAccess);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        AmpSubsystem&        m_ampSubsystem;
};
