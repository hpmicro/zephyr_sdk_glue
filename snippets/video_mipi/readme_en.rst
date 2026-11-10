.. _video_mipi_en:

MIPI-CSI Video Capture
=======================

Overview
---------

This sample captures camera frames through the HPMicro MIPI-CSI interface and presents them on the configured display output.

* `Zephyr 4.4.0 video capture sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/video/capture/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/drivers/video/capture

Hardware setup
---------------

Connect the supported MIPI-CSI camera and LCD panel to the connectors selected by the hpm6800evk snippet files.

Building
---------

Run from the west workspace root. The following commands use ``hpm6800evk`` as an example:

.. code-block:: console

   west build -p always -b hpm6800evk -S video_mipi zephyr/samples/drivers/video/capture

Expected behavior
------------------

The console lists camera formats and repeatedly reports captured frame information. With the configured display attached, the LCD shows the live camera image.

Notes
------

.. note::

   Use the exact camera and panel configuration selected by the snippet; incompatible lane or pixel-format settings prevent capture.
