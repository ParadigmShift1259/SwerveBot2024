#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>
#include <unordered_map>

#include "ISubsystemAccess.h"

class GoToAzimuthCommand: public frc2::CommandHelper<frc2::Command, GoToAzimuthCommand>
{
    public:
        explicit GoToAzimuthCommand(ISubsystemAccess& subsystemAccess);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

    private:
        DriveSubsystem&        m_driveSubsystem;
        VisionSubsystem&        m_visionSubsystem;
        LEDSubsystem&        m_led;
        units::radian_t m_commandedAzimuth;

        units::radians_per_second_t m_rotInput;
        double m_yawError;

        LEDSubsystem::Color c_colorWhite = LEDSubsystem::CreateColor(255, 255, 255, 10);
};
