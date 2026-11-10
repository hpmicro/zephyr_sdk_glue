.. _video_dvp_en:

DVP Video Capture
==================

Overview
---------

This sample captures camera frames through the HPMicro parallel DVP interface and presents them on the configured display output.

* `Zephyr 4.4.0 video capture sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/video/capture/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/drivers/video/capture

Hardware setup
---------------

Connect the DVP camera module and LCD panel selected by the board overlay, with their power and ribbon cables correctly oriented.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S video_dvp zephyr/samples/drivers/video/capture

Expected behavior
------------------

The console lists video and display capabilities, then repeatedly reports captured frame numbers, sizes, and timestamps. When a display is configured, the live camera image appears on the LCD.

Notes
------

.. note::

   The visual result depends on using the camera and panel configured by the selected board files.
