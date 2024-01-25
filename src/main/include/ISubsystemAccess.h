#pragma once

#ifndef __SUBSYSTEMACCESS_H__
#define __SUBSYSTEMACCESS_H__

#include "IDriveSubsystem.h"
#include "VisionSubsystem.h"

#include <frc/DataLogManager.h>

class ISubsystemAccess
{
public:
    virtual IDriveSubsystem&        GetDrive() = 0;
    // virtual VisionSubsystem&        GetVision() = 0;

    virtual wpi::log::DataLog&      GetLogger() = 0;
};

#endif  //ndef __SUBSYSTEMACCESS_H__
