.. _shell_shell_module_en:

Shell Module
=============

Overview
---------

This sample demonstrates how to register Zephyr shell commands, subcommands, dynamic commands, dictionary arguments, and a bypass callback.

* `Zephyr 4.4.0 shell module sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/shell/shell_module/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/subsys/shell/shell_module

Hardware setup
---------------

Connect to the board console at the baud rate configured by the board.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S shell_module zephyr/samples/subsys/shell/shell_module

Expected behavior
------------------

A shell prompt appears. Running ``help`` lists the sample commands; the commands demonstrate parameters, dynamic entries, dictionary values, and bypass input handling.

Notes
------

.. note::

   Enter ``help <command>`` in the shell to inspect the arguments supported by each demonstration command.
