#pragma once

#include "protocol_data.h"
#include "type.h"

namespace sweeper
{
namespace bus
{

class VoicePrompts2AA : public ProtocolData<CarFeedBackInfo>
{

public:

    static const int32_t ID;

    void Reset() override;
    void UpdateData(uint8_t *data) override;    

    VoicePrompts2AA *send_voice_singal_status(uint8_t signal);                

private:

    uint8_t voice_signal_ = 0;

    void set_voice_signal_status_p(uint8_t *data, uint8_t signal);    
};

} // namespace bus
} // namespace sweeper
