#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class GoToElevationCommand: public frc2::CommandHelper<frc2::Command, GoToElevationCommand>
{
    public:
        explicit GoToElevationCommand(ISubsystemAccess& subsystemAccess, units::degree_t angle);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ShooterSubsystem& m_shooterSubsystem;
		
        units::degree_t m_angle;

		wpi::log::BooleanLogEntry m_logStartGoToElevationCommand;
};
