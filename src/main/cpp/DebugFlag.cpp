#include "DebugFlag.h"

int DebugFlag::m_flagCount = 0;

const int c_numRows = 5;

DebugFlag::DebugFlag(const char* name, bool defaultVal) : m_defaultVal(defaultVal)
{

}
