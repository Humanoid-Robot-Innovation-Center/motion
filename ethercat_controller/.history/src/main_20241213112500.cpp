/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */

#include <cstdio>
#include "Console.hpp"
#include "command.h"
#include "time.h"
extern "C" {
#include "ethercat.h"
}

namespace cr = CppReadline;
using ret = cr::Console::ReturnCode;


