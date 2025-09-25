Overview
===========

This repository provides C program for audio simulation and real-time recording on Linux systems, designed for use with the Micro Speech OpenAMP samples. It includes:

- **simulation_main.c**: Send audio data from WAV files to a target device via RPMsg.
- **record_main**: Capture audio using ALSA and transmit it to the target device.

Building and Running
===========

Simulation with wav files
-----------

.. code-block:: console

    # Build
    gcc simulation_main.c -o simulate -lrt -lpthread
    
    # Run 
    ./simulate [wave_file] [tty_device]
    ./simulate yes_1000ms.wav /dev/ttyRPMSG0
    
    # Detail user guide
    ./simulate --h

Recording with ALSA
-----------
alsa-lib: 1.2.14

.. code-block:: console

    # Build alsa-lib
    tar -xvjf alsa-lib-1.2.14.tar.bz2
    cd alsa-lib-1.2.14
    ./configure --host=arm-linux && make install -j$(nproc)
    
    # Build linux app
    gcc record_main.c -o record -lrt -lpthread -lasound
    
    # Run application (for ex, using Card #1 Subdevice #0 and /dev/ttyRPMSG0)
    ./record [PCM_DEVICE] [TTY_DEVICE] 
    ./record hw:1,0 /dev/ttyRPMSG0
    
    # Detail user guide
    ./record --h
    



