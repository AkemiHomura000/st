/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP | ARM 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 
// 使用TYPE-C数据线（只能充电的线无法使用）连接到核心板的TYPE-C口


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，使用TYPE-C数据线将电脑与核心板连接起来
// 
// 2.电脑上使用串口助手打开对应的串口
// 
// 4.可以在串口助手上看到如下串口信息：
//    Seekfree USB CDC Test
//    Seekfree USB CDC Test
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


uint8 gpio_status;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化
    
    // 此处编写用户代码 例如外设初始化代码等

    usb_cdc_init();                 // 初始化USB 虚拟串口

    // 此处编写用户代码 例如外设初始化代码等
    
    while(1)
    {
        usb_cdc_write_string("Seekfree USB CDC Test\n");
        
        // 接收数据可以在zf_driver_usb_cdc.c内的usb_cdc_receive_buffer_callback函数中进行接收
        system_delay_ms(500);
    }
}

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：没有对应的串口号
//      可以尝试安装E17_usb_cdc_demo\USB虚拟串口驱动文件下面的驱动文件
// 
// 问题2：波特率应该选多少呢
//      任意波特率都是可以的，并且不会影响传输的速度
// 
