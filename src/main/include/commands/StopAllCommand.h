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
        LEDSubsystem& m_led;
        DriveSubsystem& m_drive;
        VisionSubsystem& m_vision;

        LEDSubsystem::Color c_colorPink = LEDSubsystem::CreateColor(80, 10, 15 , 0);
        LEDSubsystem::Color c_colorGreen = LEDSubsystem::CreateColor(13, 80, 0, 0);

		wpi::log::BooleanLogEntry m_logStartCommand;
};
