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
        units::radian_t m_commandedAzimuth;

        units::radians_per_second_t m_rot;
        // double m_targetX;
        // double m_targetY;
};
