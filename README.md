# hsdaoh-rp2350 - High Speed Data Acquisition over HDMI
## Stream up to 75 MByte/s from your Raspberry Pi Pico2 to your PC

Using $5 USB3 HDMI capture sticks based on the MacroSilicon MS2130, this project allows to stream out up to 75 MByte/s of real time data from an RP2350 (with overclocking) to a host computer with USB3.
For more information and the host library, see the [main repository](https://github.com/steve-m/hsdaoh) and the [talk at OsmoDevcon '24](https://media.ccc.de/v/osmodevcon2024-200-low-cost-high-speed-data-acquisition-over-hdmi).

![Raspberry Pi Pico2 with MS2130 stick](https://steve-m.de/projects/hsdaoh/pico2_hsdaoh.jpg)

## Building

Make sure you have the latest version of the [pico-sdk](https://github.com/raspberrypi/pico-sdk) installed together with an appropriate compiler. You should be able to build the [pico-examples](https://github.com/raspberrypi/pico-examples).

To build hsdaoh-rp2350:

    git clone https://github.com/steve-m/hsdaoh-rp2350.git
    mkdir hsdaoh-rp2350/build
    cd hsdaoh-rp2350/build
    export PICO_SDK_PATH=/<path-to>/pico-sdk
    cmake -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2 ../
    make -j 8

After the build succeeds you can copy the resulting *.uf2 file of the application you want to run to the board.

Apart from the Pico2 with the [Pico-DVI-Sock](https://github.com/Wren6991/Pico-DVI-Sock), it also should work with the Adafruit Feather RP2350 with HSTX Port, but so far only the Pico2 was tested.

## Example applications

The repository contains a library - libpicohsdaoh - which implements the main functionality. It reads the data from a ringbuffer, and streams it out via the HSTX port.
In addition to that, the apps folder contains a couple of example applications:

### counter

This application uses the PIO to generate a 16-bit counter value which is written to a DMA ringbuffer, which is then streamed out via hsdaoh. The counter can be verified using the hsdaoh_test host application.

### internal_adc

The data from the internal ADC is streamed out via USB. Default configuration is overclocking the ADC to 3.33 MS/s. Using the USB PLL and overvolting beyond VREG_VOLTAGE_MAX, up to 7.9 MS/s can be achieved.

### external_adc

This app contains a PIO program that reads the data from a 12-bit ADC connected to GP0-GP11, outputs the ADC clock on GP20, and packs the 12 bit samples to 16-bit words to achieve maximum throughput.
It is meant to be used with cheap AD9226 ADC boards. The default setting is overclocking the RP2350 to 320 MHz and driving the ADC with a 40 MHz clock. With higher overclocking up to 50.25 MHz ADC clock can be used.
This can be used for sampling the IF of a tuner/downcoverter, as a direct-sampling HF SDR, or for capturing a video signal e.g. with [vhsdecode](https://github.com/oyvindln/vhs-decode).

![Pico2 with AD9226 ADC board](https://steve-m.de/projects/hsdaoh/rp2350_external_adc.jpg)

## Credits

hsdaoh-rp2350 is developed by Steve Markgraf, and is based on the [dvi_out_hstx_encoder](https://github.com/raspberrypi/pico-examples/tree/master/hstx/dvi_out_hstx_encoder) example, and code by Shuichi Takano [implementing the HDMI data island encoding](https://github.com/shuichitakano/pico_lib/blob/master/dvi/data_packet.cpp), required to send HDMI info frames.
