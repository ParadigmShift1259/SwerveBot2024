#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeGoToPositionCommand: public frc2::CommandHelper<frc2::Command, IntakeGoToPositionCommand>
{
    public:
        explicit IntakeGoToPositionCommand(ISubsystemAccess& subsystemAccess, double turns);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        IntakeSubsystem& m_intakeSubsystem;
		
        double m_turns;

		wpi::log::BooleanLogEntry m_logStartIntakeGoToPositionCommand;
};
