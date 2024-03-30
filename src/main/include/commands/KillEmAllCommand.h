#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class KillEmAllCommand: public frc2::CommandHelper<frc2::Command, KillEmAllCommand>
{
    public:
        explicit KillEmAllCommand(ISubsystemAccess& subsystemAccess);
		
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        ShooterSubsystem& m_shooterSubsystem;
        LEDSubsystem& m_led;
        DriveSubsystem& m_drive;
        VisionSubsystem& m_vision;

        LEDSubsystem::Color c_colorPink = LEDSubsystem::CreateColor(80, 10, 15 , 0);
        LEDSubsystem::Color c_colorGreen = LEDSubsystem::CreateColor(13, 80, 0, 0);

		wpi::log::BooleanLogEntry m_logStartCommand;
};
