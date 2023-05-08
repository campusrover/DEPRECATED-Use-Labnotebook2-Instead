# Bluetooth Headset

## The linux bluetooth struggle

There are some troubles that you may run into if you are trying to connect to bluetooth using linux or raspberry pi, so this is a guide to try and overcome those difficulties. Hopefully, it is helpful. 

## Install Pipewire

Run the following commands to install Pipewire and disable PulseAudio. 

    sudo add-apt-repository ppa:pipewire-debian/pipewire-upstream
    sudo apt update
    sudo apt install pipewire
    sudo apt install libspa-0.2-bluetooth
    sudo apt install pipewire-audio-client-libraries
    systemctl --user daemon-reload
    systemctl --user --now disable pulseaudio.service pulseaudio.socket
    systemctl --user mask pulseaudio
    systemctl --user --now enable pipewire-media-session.service

To check that Pipewire is properly installed, run 

    pactl info

If this doesn't work, try restarting Pipewire or your computer: 

    systemctl --user restart pipewire

If you get the error:   `Connection failure: Connection refused`

    systemctl --user restart pipewire-pulse

[Source for procedure](https://askubuntu.com/questions/831331/failed-to-change-profile-to-headset-head-unit)

## Steps taken to get bluetooth headset to run 

Check the status of your bluetooth: 

    sudo systemctl status bluetooth 

To connect your bluetooth device, run the following commands: 

    sudo bluetoothctl
    agent on
    default-agent
    scan on
    pair XX:XX:XX:XX
    connect XX:XX:XX:XX
    trust XX:XX:XX:XX


## Set profile

After this, run: 

    pactl list

You'll get a list of devices and the bluetooth device will be in the form of `bluez_card.84_6B_45_98_FD_8E`

From what I understand, most bluetooth headsets have two different profiles: ad2p and headset-head-unit. To use the microphone, you will need to set the card profile of your bluetooth device to `headset-head-unit`

    pactl set-card-profile bluez_card.84_6B_45_98_FD_8E headset-head-unit

Then, test whether or not the device is recording and playing properly:

    parec -d bluez_card.84_6B_45_98_FD_8E.headset-head-unit | lame -r -V0 - out.mp3
    mpg321 out.mp3

## Change default 

You can set the default input and output devices using the following commands. 

First check what sources are available: 

    pactl list short sources
    pactl list short sinks

Then set the default source and sink devices: 

    pactl set-default-source <device_name_output>
    pactl set-default-sink <device_name_input>

