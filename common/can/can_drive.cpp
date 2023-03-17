
#include "can_drive.h"

namespace sweeper
{
    namespace common
    {
        namespace can
        {
            CanDrive::CanDrive(/* args */)
            {
                gDwDeviceHandle_ = 0;
                memset(&devInfo_, 0, sizeof(CAN_DeviceInformation));
                memset(&setCanFilter_[0], 0, sizeof(SetCanFilter_t) * DEF_USE_CAN_NUM);
                enCanChannel_[CAN_CHANNEL_CAN0] = DEF_DISABLE_CAN_CHANNEL;
                enCanChannel_[CAN_CHANNEL_CAN1] = DEF_DISABLE_CAN_CHANNEL;
            }

            CanDrive::~CanDrive()
            {
                if (setCanFilter_[CAN_CHANNEL_CAN0].CanFilter != NULL)
                    delete[] setCanFilter_[CAN_CHANNEL_CAN0].CanFilter;

                if (setCanFilter_[CAN_CHANNEL_CAN1].CanFilter != NULL)
                    delete[] setCanFilter_[CAN_CHANNEL_CAN1].CanFilter;

                if (enCanChannel_[CAN_CHANNEL_CAN0] == DEF_ENABLE_CAN_CHANNEL)
                    CAN_ChannelStop(gDwDeviceHandle_, CAN_CHANNEL_CAN0);

                if (enCanChannel_[CAN_CHANNEL_CAN1] == DEF_ENABLE_CAN_CHANNEL)
                    CAN_ChannelStop(gDwDeviceHandle_, CAN_CHANNEL_CAN1);

                CAN_DeviceClose(gDwDeviceHandle_);
            }

            DWORD CanDrive::CanDrive_start(CanConfig_t canDriveConfig)
            {
                int useCanChannel = DEF_USE_CAN_NUM;

                /* 打开设备 */
                if (gDwDeviceHandle_ == 0)
                {
                    if ((gDwDeviceHandle_ = CAN_DeviceOpen(ACUSB_132B, DEF_DEV_INDEX, NULL)) == 0)
                    {
                        printf("open deivce error\n");
                        return -1;
                    }

                    if (CAN_GetDeviceInfo(gDwDeviceHandle_, &devInfo_) != CAN_RESULT_OK)
                    {
                        printf("GetDeviceInfo error\n");
                        return -1;
                    }
                    printf("--%s--\n", devInfo_.szDescription);
                    printf("\tSN:%s\n", devInfo_.szSerialNumber);
                    printf("\tCAN 通道数:%d\n", devInfo_.bChannelNumber);
                    printf("\t硬件  版本:%x\n", devInfo_.uHardWareVersion);
                    printf("\t固件  版本:%x\n\n", devInfo_.uFirmWareVersion);
                    printf("\t驱动  版本:%x\n", devInfo_.uDriverVersion);
                    printf("\t接口库版本:%x\n", devInfo_.uInterfaceVersion);
                }

                if (devInfo_.bChannelNumber > 0)
                {
                    CAN_InitConfig canInitconfig;
                    CanDrive_config(canDriveConfig, canInitconfig);

                    if (CAN_ChannelStart(gDwDeviceHandle_, canDriveConfig.useCanChannel, &canInitconfig) != CAN_RESULT_OK)
                    {
                        printf("Start CAN %d error\n", canDriveConfig.useCanChannel);
                        return -1;
                    }
                    else
                    {
                        enCanChannel_[canDriveConfig.useCanChannel] = DEF_ENABLE_CAN_CHANNEL;
                        printf("Start CAN %d success\n", canDriveConfig.useCanChannel);
                    }
                }

                return canDriveConfig.useCanChannel;
            }

            void CanDrive::CanDrive_config(CanConfig_t config, CAN_InitConfig &canInitconfig)
            {
                canInitconfig.dwAccCode = 0;
                canInitconfig.dwAccMask = 0xffffffff;
                canInitconfig.nFilter = config.nFilter;   // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
                canInitconfig.bMode = config.bMode;       // 工作模式(0表示正常模式,1表示只听模式)
                canInitconfig.nBtrType = config.nBtrType; // 位定时参数模式(1表示SJA1000,0表示LPC21XX)

                // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
                // BTR1
                switch (config.baudrate)
                {
                case CAN_BAUDRATE_5K:
                    canInitconfig.dwBtr[0] = 0xBF;
                    canInitconfig.dwBtr[1] = 0xFF;
                    break;
                case CAN_BAUDRATE_10K:
                    canInitconfig.dwBtr[0] = 0x31;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_20K:
                    canInitconfig.dwBtr[0] = 0x18;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_50K:
                    canInitconfig.dwBtr[0] = 0x09;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_100K:
                    canInitconfig.dwBtr[0] = 0x04;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_125K:
                    canInitconfig.dwBtr[0] = 0x03;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_250K:
                    canInitconfig.dwBtr[0] = 0x01;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_500K:
                    canInitconfig.dwBtr[0] = 0x00;
                    canInitconfig.dwBtr[1] = 0x1C;
                    break;
                case CAN_BAUDRATE_800K:
                    canInitconfig.dwBtr[0] = 0x00;
                    canInitconfig.dwBtr[1] = 0x16;
                    break;
                case CAN_BAUDRATE_1000K:
                    canInitconfig.dwBtr[0] = 0x00;
                    canInitconfig.dwBtr[1] = 0x14;
                    break;
                default:
                    printf("input baudrate error!\n");
                    break;
                }
                canInitconfig.dwBtr[2] = 0;
                canInitconfig.dwBtr[3] = 0;
            }

            DWORD CanDrive::CanDrive_canFilter(DWORD dwChanne, CanFilter_t *canFilter, uint32_t canIdNum)
            {
                if (dwChanne == CAN_CHANNEL_CAN0 || dwChanne == CAN_CHANNEL_CAN1)
                {
                    setCanFilter_[dwChanne].filterFlag = DEF_USE_CAN_FILTER;
                    setCanFilter_[dwChanne].useCanChannel = dwChanne;
                    setCanFilter_[dwChanne].canFilterNum = canIdNum;
                    setCanFilter_[dwChanne].CanFilter = new CanFilter_t[canIdNum];
                    if (setCanFilter_[dwChanne].CanFilter == NULL)
                    {
                        printf("canFilter申请空间失败");
                        return -1;
                    }
                    for (int32_t i = 0; i < canIdNum; i++)
                    {
                        setCanFilter_[dwChanne].CanFilter[i].can_id = canFilter[i].can_id;
                        setCanFilter_[dwChanne].CanFilter[i].bRemoteFlag = canFilter[i].bRemoteFlag;
                        setCanFilter_[dwChanne].CanFilter[i].bExternFlag = canFilter[i].bExternFlag;
                        setCanFilter_[dwChanne].CanFilter[i].nSendType = canFilter[i].nSendType;
                    }
                }
                else
                {
                    printf("输入can通道错误, 请确认后重新输入");
                }

                return 0;
            }

            uint32_t CanDrive::CanDrive_read(DWORD dwChannel, can_frame *canDataFrame)
            {
                int reclen = 0;
                CAN_DataFrame rec[20];
                CAN_ErrorInformation err;

                while ((dwChannel == CAN_CHANNEL_CAN0 || dwChannel == CAN_CHANNEL_CAN1))
                {
                    if ((reclen = CAN_ChannelReceive(gDwDeviceHandle_, dwChannel, rec, __countof(rec), 200) > 0))
                    {
                        if (setCanFilter_[dwChannel].filterFlag == DEF_USE_CAN_FILTER)
                        {
                            if (CanDrive_matchFilter(dwChannel, rec[reclen - 1]) != 1)
                            {
                                continue;
                            }
                        }
                        canDataFrame->can_id = rec[reclen - 1].uID;
                        canDataFrame->can_dlc = rec->nDataLen;
                        for (int i = 0; i < rec[reclen - 1].nDataLen; i++)
                        {
                            canDataFrame->data[i] = rec[reclen - 1].arryData[i];
                        }
                        return canDataFrame->can_dlc;
                    }
                    else
                    {
                        /* 必须调用CAN_GetErrorInfo函数处理错误信息 */
                        if (CAN_GetErrorInfo(gDwDeviceHandle_, dwChannel, &err) == CAN_RESULT_OK)
                        {
                            printf("file:%s function:%s line:%d\n",__FILE__, __FUNCTION__, __LINE__);
                            return 0;
                        }
                        else
                        {
                            continue;   // CAN卡没有收到CAN报文
                        }
                    }
                }
                return 0;
            }

            uint32_t CanDrive::CanDrive_matchFilter(DWORD dwChannel, CAN_DataFrame canData)
            {
                uint32_t matchFlag = 0;
                for (uint32_t index = 0; index < setCanFilter_[dwChannel].canFilterNum; index++)
                {
                    if (canData.uID != setCanFilter_[dwChannel].CanFilter[index].can_id)
                        continue;

                    if (canData.bExternFlag != setCanFilter_[dwChannel].CanFilter[index].bExternFlag)
                        continue;

                    if (canData.bRemoteFlag != setCanFilter_[dwChannel].CanFilter[index].bRemoteFlag)
                        continue;

                    matchFlag = 1;
                    break;
                }
                return matchFlag;
            }

            uint32_t CanDrive::CanDrive_write(DWORD dwChannel, can_frame *canDataFrame, uint8_t SendType, uint8_t bRemoteFlag, uint8_t bExternFlag, uint8_t sndFrames)
            {
                unsigned long sndCnt = 0;
                CAN_DataFrame *send = new CAN_DataFrame[sndFrames];

                for (int j = 0; j < sndFrames; j++)
                {
                    send[j].uID = canDataFrame->can_id;
                    send[j].nDataLen = canDataFrame->can_dlc;
                    send[j].bExternFlag = bExternFlag;
                    send[j].bRemoteFlag = bRemoteFlag;
                    send[j].nSendType = SendType;
                    for (int i = 0; i < send[j].nDataLen; i++)
                    {
                        send[j].arryData[i] = canDataFrame->data[i];
                    }
                }
                sndCnt = CAN_ChannelSend(gDwDeviceHandle_, dwChannel, send, sndFrames);
                if(0 == sndCnt)
                {
                    printf("send fail %d\n",__LINE__);
                }

                if(send != nullptr)
                    delete[] send;

                return canDataFrame->can_dlc * sndCnt;
            }
        }
    }
}