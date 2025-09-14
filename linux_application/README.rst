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
alsa-lib: 1.12.14

.. code-block:: console

    # Build
    gcc record_main.c -o send -lrt -lpthread -lasound
    # Run 
    ./send
