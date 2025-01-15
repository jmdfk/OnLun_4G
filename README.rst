Supported Hardware
******************

This firmware can be built for a variety of supported hardware platforms.

.. pull-quote::
   [!IMPORTANT]

   In Zephyr, each of these different hardware variants is given a unique
   "board" identifier, which is used by the build system to generate firmware
   for that variant.

   When building firmware using the instructions below, make sure to use the
   correct Zephyr board identifier that corresponds to your follow-along
   hardware platform.

.. list-table:: **Follow-Along Hardware**
   :header-rows: 1

   * - Hardware
     - Zephyr Board
    

   * - .. image:: images/can_asset_tracker_fah_nrf9160_dk.jpg
          :width: 240
     - ``nrf9160dk_nrf9160_ns``
    


Firmware Overview
*****************

The vehicle sensor values are combined with GPS location/time data and uploaded
to the Golioth Cloud. The timestamp from the GPS reading is used as the
timestamp for the data record in the Golioth LightDB Stream database.

GPS readings can be received as frequently as once-per-second. When the device
is out of cellular range, the firmware caches data locally and
uploads it later when connection to the cellular network is restored.

Supported Golioth Zephyr SDK Features
=====================================

This firmware implements the following features from the Golioth Zephyr SDK:

- `Device Settings Service <https://docs.golioth.io/firmware/zephyr-device-sdk/device-settings-service>`_
- `LightDB State Client <https://docs.golioth.io/firmware/zephyr-device-sdk/light-db/>`_
- `LightDB Stream Client <https://docs.golioth.io/firmware/zephyr-device-sdk/light-db-stream/>`_
- `Logging Client <https://docs.golioth.io/firmware/zephyr-device-sdk/logging/>`_
- `Over-the-Air (OTA) Firmware Upgrade <https://docs.golioth.io/firmware/device-sdk/firmware-upgrade>`_
- `Remote Procedure Call (RPC) <https://docs.golioth.io/firmware/zephyr-device-sdk/remote-procedure-call>`_

Device Settings Service
-----------------------

The following settings can be set in the Device Settings menu of the `Golioth
Console`_.

``LOOP_DELAY_S``
   Adjusts the delay between sensor readings. Set to an integer value (seconds).

   Default value is ``5`` seconds.

``GPS_DELAY_S``
   Adjusts the delay between recording GPS readings. Set to an integer value
   (seconds).

   Default value is ``3`` seconds.

``FAKE_GPS_ENABLED``
   Controls whether fake GPS position data is reported when a real GPS location
   signal is unavailable. Set to a boolean value.

   Default value is ``false``.

``FAKE_GPS_LATITUDE``
   Sets the fake latitude value to be used when fake GPS is enabled. Set to a
   floating point value (``-90.0`` to ``90.0``).

   Default value is ``37.789980``.

``FAKE_GPS_LONGITUDE``
   Sets the fake longitude value to be used when fake GPS is enabled. Set to a
   floating point value (``-180.0`` to ``180.0``).

   Default value is ``-122.400860``.

``VEHICLE_SPEED_DELAY_S``
   Adjusts the delay between vehicle speed readings. Set to an integer value
   (seconds).

   Default value is ``1`` second.

LightDB Stream Service
----------------------

Vehicle data is periodically sent to the following endpoints of the LightDB
Stream service:

* ``gps/lat``: Latitude (°)
* ``gps/lon``: Longitude (°)
* ``gps/fake``: ``true`` if GPS location data is fake, otherwise ``false``
* ``vehicle/speed``: Vehicle Speed (km/h)

LightDB State Service
---------------------

The concept of Digital Twin is demonstrated with the LightDB State
``example_int0`` and ``example_int1`` variables that are members of the
``desired`` and ``state`` endpoints.

* ``desired`` values may be changed from the cloud side. The device will
  recognize these, validate them for [0..65535] bounding, and then reset these
  endpoints to ``-1``

* ``state`` values will be updated by the device whenever a valid value is
  received from the ``desired`` endpoints. The cloud may read the ``state``
  endpoints to determine device status, but only the device should ever write to
  the ``state`` endpoints.

Remote Procedure Call (RPC) Service
-----------------------------------

The following RPCs can be initiated in the Remote Procedure Call menu of the
`Golioth Console`_.

``get_network_info``
   Query and return network information.

``reboot``
   Reboot the system.

``set_log_level``
   Set the log level.

   The method takes a single parameter which can be one of the following integer
   values:

   * ``0``: ``LOG_LEVEL_NONE``
   * ``1``: ``LOG_LEVEL_ERR``
   * ``2``: ``LOG_LEVEL_WRN``
   * ``3``: ``LOG_LEVEL_INF``
   * ``4``: ``LOG_LEVEL_DBG``

Building the firmware
*********************

The firmware build instructions below assume you have already set up a Zephyr
development environment and have some basic familiarity with building firmware
using the Zephyr Real Time Operating System (RTOS).

If you're brand new to building firmware with Zephyr, you will need to follow
the `Zephyr Getting Started Guide`_ to install the Zephyr SDK and related
dependencies.

We also provide free online `Developer Training`_ for Zephyr at:

https://training.golioth.io/docs/zephyr-training

.. pull-quote::
   [!IMPORTANT]

   Do not clone this repo using git. Zephyr's ``west`` meta-tool should be used
   to set up your local workspace.

Create a Python virtual environment (recommended)
=================================================

.. code-block:: shell

   cd ~
   mkdir Modz2
   python -m venv Modz2/.venv
   source Modz2/.venv/bin/activate

Install ``west`` meta-tool
==========================

.. code-block:: shell

   pip install wheel west

Use ``west`` to initialize the workspace and install dependencies
=================================================================

.. code-block:: shell

   cd ~/Modz2
   west init -m git@github.com:scottbitbot2502/ModzV2.git .
   west update
   west zephyr-export
   pip install -r deps/zephyr/scripts/requirements.txt

Build the firmware
==================

Build the Zephyr firmware from the top-level workspace of your project. After a
successful build you will see a new ``build/`` directory.

Note that this git repository was cloned into the ``app`` folder, so any changes
you make to the application itself should be committed inside this repository.
The ``build`` and ``deps`` directories in the root of the workspace are managed
outside of this git repository by the ``west`` meta-tool.

Prior to building, update ``CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION`` in the
``prj.conf`` file to reflect the firmware version number you want to assign to
this build.

.. pull-quote::
   [!IMPORTANT]

   When running the commands below, make sure to replace the placeholder
   ``<your_zephyr_board_id>`` with the actual Zephyr board from the table above
   that matches your follow-along hardware.

.. code-block:: text

   $ (.venv) west build -p -b <your_zephyr_board_id> app

For example, to build firmware for the `Nordic nRF9160 DK`_-based follow-along
hardware:

.. code-block:: text

   $ (.venv) west build -p -b nrf9160dk_nrf9160_ns app

Flash the firmware
==================

.. code-block:: text

   $ (.venv) west flash

Provision the device
====================

In order for the device to securely authenticate with the Golioth Cloud, we need
to provision the device with a pre-shared key (PSK). This key will persist
across reboots and only needs to be set once after the device firmware has been
programmed. In addition, flashing new firmware images with ``west flash`` should
not erase these stored settings unless the entire device flash is erased.

Configure the PSK-ID and PSK using the device UART shell and reboot the device:

.. code-block:: text

   uart:~$ settings set golioth/psk-id <my-psk-id@my-project>
   uart:~$ settings set golioth/psk <my-psk>
   uart:~$ kernel reboot cold

External Libraries
******************

The following code libraries are installed by default. If you are not using the
custom hardware to which they apply, you can safely remove these repositories
from ``west.yml`` and remove the includes/function calls from the C code.

* `golioth-zephyr-boards`_ includes the board definitions for the Golioth
  Aludel-Mini
* `libostentus`_ is a helper library for controlling the Ostentus ePaper
  faceplate
* `zephyr-network-info`_ is a helper library for querying, formatting, and
  returning network connection information via Zephyr log or Golioth RPC
