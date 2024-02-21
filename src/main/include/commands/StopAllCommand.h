#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class StopAllCommand: public frc2::CommandHelper<frc2::Command, StopAllCommand>
{
    public:
        explicit StopAllCommand(ISubsystemAccess& subsystemAccess);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ShooterSubsystem& m_shooterSubsystem;
        IntakeSubsystem& m_intakeSubsystem;

		wpi::log::BooleanLogEntry m_logStartCommand;
};
