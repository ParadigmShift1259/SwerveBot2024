#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ShootCommand: public frc2::CommandHelper<frc2::Command, ShootCommand>
{
    public:
        explicit ShootCommand(ISubsystemAccess& subsystemAccess);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ShooterSubsystem& m_shooterSubsystem;
        IntakeSubsystem& m_intakeSubsystem;
        frc::Timer m_timer;
        
		wpi::log::BooleanLogEntry m_logStartShootCommand;
};
