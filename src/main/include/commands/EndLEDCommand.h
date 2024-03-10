#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class EndLEDCommand: public frc2::CommandHelper<frc2::Command, EndLEDCommand>
{
    public:
        explicit EndLEDCommand(ISubsystemAccess& subsystemAccess);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        IntakeSubsystem& m_intakeSubsystem;
        LEDSubsystem& m_led;

        LEDSubsystem::Color c_colorPink = LEDSubsystem::CreateColor(80, 10, 15 , 0);
        LEDSubsystem::Color c_colorGreen = LEDSubsystem::CreateColor(13, 80, 0, 0);
        frc::Timer m_timer;

		wpi::log::BooleanLogEntry m_logStartEndLEDCommand;
};
