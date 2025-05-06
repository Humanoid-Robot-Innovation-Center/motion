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

int main()
{
    printf("SOEM 主站测试\n");

    // 这里填自己电脑上的网卡
    EtherCAT_Init("enp3s0");//enx68da73a9766a:

    if (ec_slavecount <= 0)
    {
        printf("未找到从站, 程序退出！");
        return 1;
    }
    else
        printf("从站数量： %d\r\n", ec_slavecount);

    while (1)
    {

             EtherCAT_Run();
             usleep(1000);
        
    //    usleep(10000);
    //    for(j= 0;j++;j<50000);
    }

}
