#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ClimbCommand: public frc2::CommandHelper<frc2::Command, ClimbCommand>
{
    public:
        explicit ClimbCommand(ISubsystemAccess& subsystemAccess, ClimberSubsystem::Position pos);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ClimberSubsystem&        m_climber;
        LEDSubsystem& m_led;

        LEDSubsystem::Color c_colorPink = LEDSubsystem::CreateColor(80, 10, 15 , 0);
        LEDSubsystem::Color c_colorGreen = LEDSubsystem::CreateColor(13, 80, 0, 0);
        frc::Timer m_timer;

        ClimberSubsystem::Position m_position;
};
