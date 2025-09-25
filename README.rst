Building and Running
===========

Simulation with wave files
-----------

.. code-block:: console

    # Build
    gcc simulation_main.c -o send -lrt -lpthread
    # Run 
    ./send <file_name>.wav
    
    # User guide
    ./send --h

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
    
    # Run application (for ex, using Card #1, Subdevice #0)
    ./record hw:1,0 
    
    # User guide
    ./record --h
    



