#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ShootCommand: public frc2::CommandHelper<frc2::Command, ShootCommand>
{
    public:
        explicit ShootCommand(ISubsystemAccess& subsystemAccess, units::meter_t distance);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ShooterSubsystem& m_shooterSubsystem;
        IntakeSubsystem& m_intakeSubsystem;
		
        units::meter_t m_distance;

		wpi::log::BooleanLogEntry m_logStartShootCommand;
};
