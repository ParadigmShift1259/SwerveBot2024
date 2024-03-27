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
        LEDSubsystem& m_led;
        VisionSubsystem& m_vision;

        LEDSubsystem::Color c_colorPink = LEDSubsystem::CreateColor(80, 10, 15 , 0);
        LEDSubsystem::Color c_colorGreen = LEDSubsystem::CreateColor(13, 80, 0, 0);
		
        units::meter_t m_distance;
        units::degree_t m_elevationAngle;

		wpi::log::BooleanLogEntry m_logStartPreShootCommand;
};
