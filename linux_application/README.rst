Build steps
===========

Simulation
-----------

.. code-block:: console

    # Build
    gcc simulation_main.c -o send -lrt -lpthread
    # Run 
    ./send <file_name>.wav

Using ALSA
-----------
alsa-lib: `1.2.14 <http://www.alsa-project.org/files/pub/lib/alsa-lib-1.2.14.tar.bz2>`_

.. code-block:: console
    # Build alsa-lib
    tar -xvjf alsa-lib-1.2.14.tar.bz2
    cd alsa-lib-1.2.14
    ./configure --host=arm-linux && make install -j$(nproc)

    # Build linux app
    gcc record_main.c -o record -lrt -lpthread -lasound
    # Get the soundcard information
    arecord -l
    
    # User guide
    ./record --h
    # Run application (for ex, using Card #1, Subdevice #0)
    ./record hw:1,0
