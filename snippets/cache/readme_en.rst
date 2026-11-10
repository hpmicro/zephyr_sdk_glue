.. _cache_en:

Cache API Test
===============

Overview
---------

This sample runs Zephyr's kernel cache test on HPMicro hardware to exercise cache flush, invalidate, and data-coherency behavior.

Application paths
------------------

.. code-block:: text

   zephyr/tests/kernel/cache

Hardware setup
---------------

No external hardware is required.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S cache zephyr/tests/kernel/cache

Expected behavior
------------------

The ztest console reports each cache test case and finishes with a passing summary when cache maintenance works correctly.

Notes
------

.. note::

   Zephyr does not provide a separate HTML sample page for this test; the link above points to the official v4.4.0 test source.
