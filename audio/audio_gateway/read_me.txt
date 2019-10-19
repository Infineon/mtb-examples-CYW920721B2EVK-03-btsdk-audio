-------------------------------------------------------------------------------
Audio Gateway app
-------------------------------------------------------------------------------

Overview
--------
This app demonstrates use of Bluetooth Audio Gateway profile.

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the WICED board
2. Use ClientControl application to send various commands


BR/EDR
- To find BR/EDR devices: Click on "Start BR/EDR Discovery"


AG Connection
- To create audio gateway connection to remote handsfree controller, choose the bluetooth
  address of the remote device from the BR/EDR combo box
- Click "Connect" button under AG
- To establish SCO connection, click on Audio connect
- Check SCO audio quality
- NOTE : Default WBS is disabled, update hf_control_esco_params structure to enabled it.
-------------------------------------------------------------------------------