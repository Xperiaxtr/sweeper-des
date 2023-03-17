#pragma once
#include <iostream>
#include <cstdlib>
#include <string.h>
#include <pthread.h>
#include <thread>
#include <vector>
#include "third_party/ICANCmd.h"
#include <linux/can.h>

namespace sweeper
{
    namespace common
    {
        namespace can
        {
            #define DEF_DEV_INDEX 0              // CAN设备索引,从0开始
            #define DEF_USE_CAN_NUM 2            // CAN 通道数
            #define DEF_USE_CAN_FILTER 1         // 过滤功能 0 - 未使用；1使用
            #define DEF_ENABLE_CAN_CHANNEL 1     // CAN通道使能标志
            #define DEF_DISABLE_CAN_CHANNEL (-1) // CAN通道未使能标志
            #define DEF_SEND_TYPE 2              // CAN发送类型,0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
            #define DEF_SEND_FRAMES 64           // 每次发送帧数
            #define DEF_SEND_TIMES 1000          // 发送次数
            #define DEF_SEND_DELY 5              // 发送前延时,单位秒
            #define __countof(a) (sizeof(a) / sizeof(a[0]))

            enum CAN_Baudrate
            {
                CAN_BAUDRATE_5K = 5,      // 5Kbps
                CAN_BAUDRATE_10K = 10,    // 10Kbps
                CAN_BAUDRATE_20K = 20,    // 20Kbps
                CAN_BAUDRATE_50K = 50,    // 50Kbps
                CAN_BAUDRATE_100K = 100,  // 100Kbps
                CAN_BAUDRATE_125K = 125,  // 125Kbps
                CAN_BAUDRATE_250K = 250,  // 250Kbps
                CAN_BAUDRATE_500K = 500,  // 500Kbps
                CAN_BAUDRATE_800K = 800,  // 800Kbps
                CAN_BAUDRATE_1000K = 1000 // 1000Kbps
            };

            enum CAN_Filter
            {
                CAN_UNFILERED = 0,    // 0表示未设置滤波功能
                CAN_DUAL_FILTER = 1,  // 1表示双滤波
                CAN_SINGLE_FILTER = 2 // 2表示单滤波
            };

            /* 工作模式( */
            enum CAN_Mode
            {
                CAN_MODE_NORMAL = 0,     // 0表示正常模式
                CAN_MODE_JUST_LISTEN = 1 // 1表示只听模式
            };

            /* 位定时参数模式 */
            enum CAN_BtrType
            {
                CAN_BTRTYPE_LPC21XX = 0, // 0表示LPC21XX
                CAN_BTRTYPE_SJA1000 = 1  // 1表示SJA1000
            };

            /* 发送帧类型 */
            enum CAN_SendType
            {
                CAN_SEND_TYPE_NORMAL = 0,    // 0-正常发送
                CAN_SEND_TYPE_SINGLE = 1,    // 1-单次发送
                CAN_SEND_TYPE_ASR = 2,       // 2-自发自收
                CAN_SEND_TYPE_SINGLE_ASR = 3 // 3-单次自发自收
            };

            /*远程帧标志*/
            enum CAN_RemoteFlag
            {
                CAN_DATA_FRAME_FLAG = 0,   // 数据帧
                CAN_REMOTER_FRAME_FLAG = 1 // 远程帧
            };

            /* 帧格式 */
            enum CAN_ExternFlag
            {
                CAN_STANDARD_FRAME_FLAG = 0, // 标准帧
                CAN_EXTERN_FRAME_FLAG = 1    // 扩展帧
            };

            /* can 通道 */
            enum CAN_Channel
            {
                CAN_CHANNEL_CAN0 = 0, // 0表示can0
                CAN_CHANNEL_CAN1 = 1  // 1表示can1
            };

            typedef struct CanConfig
            {
                uint32_t useCanChannel; // 使用的can通道 0表示can0, 1表示can1
                uint32_t baudrate;      // 波特率
                uint32_t nFilter;       // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
                uint32_t bMode;         // 工作模式(0表示正常模式,1表示只听模式)
                uint32_t nBtrType;      // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
            } CanConfig_t;

            typedef struct CanDriveHead
            {
                uint8_t useCanChannel; // 使用的can通道 0表示can0, 1表示can1
                uint8_t initFlag;      // 初始化成功标志
            } CanDriveHead_t;

            typedef struct CanDataFrame
            {
                /* data */
                uint32_t can_id;                             // can id(uID)
                uint8_t dataLen;                             // 数据长度(<=8),也就是 arryData 的长度
                uint8_t data[8] __attribute__((aligned(8))); // can 报文数据
            } CanDataFrame_t;

            typedef struct CanFilter
            {
                uint32_t can_id;
                uint8_t nSendType;   // 发送帧类型,0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
                uint8_t bRemoteFlag; // 是否是远程帧,0 表示数据帧,1 表示远程帧
                uint8_t bExternFlag; // 是否是扩展帧,0 表示标准帧,1 表示扩展帧
            } CanFilter_t;

            typedef struct SetCanFilter
            {
                uint8_t filterFlag;    // 使用过滤功呢
                uint8_t useCanChannel; // 使用的can通道 0表示can0, 1表示can1
                uint32_t canFilterNum; // 过滤canId个数;
                CanFilter_t *CanFilter;
            } SetCanFilter_t;

            class CanDrive
            {
            private:
                void CanDrive_config(CanConfig_t config, CAN_InitConfig &canInitconfig);

                uint32_t CanDrive_matchFilter(DWORD dwChannel, CAN_DataFrame canData);

                CAN_DeviceInformation devInfo_;

                SetCanFilter_t setCanFilter_[DEF_USE_CAN_NUM];

                DWORD gDwDeviceHandle_; // 设备句柄

                int8_t enCanChannel_[DEF_USE_CAN_NUM];

                CanDriveHead_t canDriveHead_[2]; // 索引0 - 表示can0, 索引1 - 表示can1

            public:
                CanDrive(/* args */);

                ~CanDrive();

                DWORD CanDrive_start(CanConfig_t config);

                DWORD CanDrive_canFilter(DWORD dwChanne, CanFilter_t *canFilter, uint32_t canIdNum);

                uint32_t CanDrive_read(DWORD dwChannel, can_frame *canDataFrame);

                uint32_t CanDrive_write(DWORD dwChannel, can_frame *canDataFrame, uint8_t SendType = 0, uint8_t bRemoteFlag = 0, uint8_t bExternFlag = 0,  uint8_t sndFrames = 1);
            };
        }
    }
}