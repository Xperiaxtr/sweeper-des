#ifndef USBCANII_H
#define USBCANII_H
#include <QThread>
#include <QMetaType>


typedef struct
{
    QString Bandstr;
    int btr0;
    int btr1;
}s_USB_CAN_BANDRATE_BTR;

class UsbCanII : public QObject
{
    Q_OBJECT
public:
    UsbCanII() ;
    virtual ~UsbCanII();


public :
    int m_Status;
    int m_DeviceType;
    int m_DeviceIndex;
    int m_DeviceChannel;

    int m_FileSize;

public :
    CanDeviceData();
    CanDeviceData(int, int ,int ,int, int,int ,int);

    // 需要界面传递的参数
    int m_BandRateSel;
    int mCanId;
    QString mfilename;

public :
    void Run();
    OpenDevice();
    TransmitFile();
    TransmitFrame(VCI_CAN_OBJ *pSendBuf);
    Receive();

private :
    s_USB_CAN_BANDRATE_BTR m_BandrateBtr[13]
    {
        {"10Kbps",0x31,0x1c},
        {"20Kbps",0x18,0x1c},
        {"40Kbps",0x87,0xFF},
        {"50Kbps",0x09,0x1c},
        {"80Kbps",0x83,0xFF},
        {"100Kbps",0x04,0x1c},
        {"125Kbps",0x03,0x1c},
        {"200Kbps",0x81,0xFA},
        {"250Kbps",0x01,0x1C},
        {"400Kbps",0x80,0xFA},
        {"500Kbps",0x00,0x1c},
        {"800Kbps",0x00,0x16},
        {"1000Kbps",0x00,0x14},
    };
};

//Q_DECLARE_METATYPE(UsbCanData)   //一定要添加这个宏 ！！！！很重要！！！！


#endif // USBCANII_H
