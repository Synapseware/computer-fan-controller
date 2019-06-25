#ifndef __ADC_H__
#define __ADC_H__


static void ConfigureADCChannel(uint8_t channel)
{
    channel     &=  0x0F;
    uint8_t mux =   (ADMUX & 0xF0) |    // mask out the channel bits
                    (channel);          // set the channel

    if (channel < 8)
    {
        // disable digital input on the selected channel
        DIDR0 = (1<<channel);

        // set the pin as input
        DDRC &= ~(1<<channel);
    }
    else if (0x08 == channel) // 8
    {
        // internal temperature sensor
    }
    else if (0x0E == channel) // 14
    {
        // internal 1.1v band gap reference
    }
    else if (0x0F == channel) // 15
    {
        // ground
    }
    else
    {
        // invalid channel selection
        return;
    }

    // set the MUX register
    ADMUX = mux;
}


#endif
