#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class PreShootCommand: public frc2::CommandHelper<frc2::Command, PreShootCommand>
{
    public:
        explicit PreShootCommand(ISubsystemAccess& subsystemAccess);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ShooterSubsystem& m_shooterSubsystem;
		
		wpi::log::BooleanLogEntry m_logStartPreShootCommand;
};
