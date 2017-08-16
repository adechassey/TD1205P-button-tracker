Welcome to TD RF Module SDK Tools (powered by Telecom Design)
=============================================================

The TD RF Module SDK Tools provide a complete Rapid Embedded Development environment for TD RF modules.

The TD RF Module SDK Tools consist in the following components:

  - TD RF Module SDK IDE, an Eclipse-based Integrated Development Environment (IDE) 
  - GNU-based C/C++ compilation tools and documentation
  - Silicon Labs/EnergyMicro tools for designing, profiling and flashing the Silicon Labs/EnergyMicro EFM32 range of ARM Cortex M3 CPUs
  - Segger J-Link ARM JTAG/SWD adapter driver, flash utilities and debug server
  - FTDI VCP (Virtual COM Port) driver for LVTTL UART to USB adapter cable
  - PuTTY serial terminal emulator and telnet/SSH client
  - Portable Git version control system
  - Stand-Alone Windows/Apache/Mysql/PHP (WAMP) server for development of web services
  - Command line utilities

The TD RF Module SDK Tools run on Microsoft Windows platforms (XP SP2 and later), both 32-bit and 64-bit.

Further details on the TD RF Module SDK can be found at <https://developers.insgroup.fr/>

We hope you will enjoy using the TD RF Module SDK Tools!

_The Telecom Design Team_

Installation
============

This zip file include a ready-to-use, portable Eclipse environment which does not require any Eclipse or plugin installation. Nevertheless a couple of 
additional steps must be followed before making use of this environment.

  - Unpack the provided archive into the `"C:\"` folder. Please do not use any other folder to avoid issues when using the Eclipse environment.
  - Navigate to the `"C:\TD\TD_RF_Module_SDK-v4.0.0\segger\USBDriver"` folder and double-click on the `"InstDrivers.exe"` icon: this will install the required J-Link JTAG drivers
  - Follow the procedure corresponding to your platform to [install the FTDI VCP driver](http://www.ftdichip.com/Support/Documents/InstallGuides.htm), selecting the `"C:\TD\TD_RF_Module_SDK-v4.0.0\ftdi\CDM v2.08.28 Certified"` as the folder containing the driver
  - Add the compilation tools path to the environment path:
    - **Windows 2000 and Windows XP users**
        1. From the Desktop, right-click `"My Computer"` and click `"Properties"`.
        2. In the `"System Properties"` window, click on the `"Advanced"` tab.
        3. In the `"Advanced"` section, click the `"Environment Variables"` button.
        4. Finally, in the `"Environment Variables"` window, highlight the `"Path"` variable in the `"Systems Variable"` section and click on the `"Edit"` button.
        5. Add `"C:\TD\TD_RF_Module_SDK-v4.0.0\gnu\bin;"` in front of the existing path and click on the `"OK"` button twice.
    - **Windows Vista, Windows 7 and Windows 8 users**
        1. From the Desktop, right-click `"My Computer"` and click `"Properties"`.
        2. Click `"Advanced System Settings"` link in the left column.
        3. In the `"System Properties"` window click the `"Environment Variables"` button.
        4. Finally, in the `"Environment Variables"` window, highlight the `"Path"` variable in the `"Systems Variable"` section and click on the `"Edit"` button.
        5. Add the `"C:\TD\TD_RF_Module_SDK-v4.0.0\gnu\bin;"` in front of the existing path and click on the `"OK"` button twice.

The first time you plug the Silicon Labs/Energy Micro Starter Kit board or the provided FTDI cable into your PC, you may be prompted for installing the corresponding drivers.

Proceed as usual for your operating system, opting for installing the drivers from a local directory:

  - `"C:\TD\TD_RF_Module_SDK-v4.0.0\segger\USBDriver"`  for the Silicon Labs/Energy Micro Starter Kit board
  - `"C:\TD\TD_RF_Module_SDK-v4.0.0\ftdi"` for the FTDI cable

Getting the Sources
===================

The SDK source code and examples are not distributed with this zip file but are available through our Github repository once you register your Telecom Design Evaluation Board (EVB) on <https://developers.insgroup.fr/>.

The following steps detail how to download the source code and import all projects into the Eclipse environment.

  1. Navigate to the `"C:\TD\TD_RF_Module_SDK-v4.0.0\eclipse"` folder and double-click on the `"eclipse.exe"` icon
  2. Open the `"File"` menu and select the `"Import..."` item.
  3. In the `"Import"` dialog, unfold the `"Git"` folder by clicking on the `"+"` sign left to it, select the `"Projects from Git"` item and click on the `"Next >"` button
  4. In the `"Import Projects from Git"`, select the `"URI"` icon ad click on the `"Next >"` button
  5. Enter `The Github repository URL "https://github.com/Telecom-Design/TD_RF_Module_SDK.git"`in the `"URI"` field
  6. Enter your Github username and password in the `"User:"` and `"Password:"` fields, respectively and click on the `"Next >"` button
  7. Check the `"Master"` branch box and click on the `"Next >"` button
  8. Enter `"C:\TD\TD_RF_Module_SDK-v4.0.0\Github\TD_RF_Module_SDK"` in the `"Directory:"` field and click on the `"Next >"` button
  9. Check the `"Import existing projects"` radio button and click on the `"Next >"` button
  10. Click on the `"Finish"` button. The Git import will take place, this may take a while

All the available libraries and examples should now be available in the Project Explorer panel.

Organizing the Sources
======================

By default, al projects are presented in Eclipse at the same level without particular organization except that they are sorted alphabetically.

In order to have a more logical organization, we must import an Eclipse "Working Set" that will provide a grouping of projects by categories.

To do so, launch Eclipse by navigating to the `"C:\TD\TD_RF_Module_SDK-v4.0.0\eclipse"` folder and double-click on the `"eclipse.exe"` icon (if not already done) and:

  1. Open the `"File"` menu and select the `"Import..."` item.
  2. In the `"Import"` dialog, unfold the `"General"` folder by clicking on the `"+"` sign left to it, select the `"Working Sets"` item and click on the `"Next >"` button
  3. Enter `"C:\TD\TD_RF_Module_SDK-v4.0.0\Github\TD_RF_Module_SDK\TD_RF_Module_SDK.wst"` in the `"Browse..."` field, check all working sets and click on the `"Finish"` button
  4. Click on the small downwards arrow in the top-right corner of the `"Project Explorer"` panel and select the `"Select Working Sets..."` item
  5. In the `"Select Working Sets"` dialog, click on the `"Select All"`, then on the `"OK"` button
  6. Click on the small downwards arrow in the top-right corner of the `"Project Explorer"` panel again and select the `"Top Level Elements > Working Sets"` item

All the available libraries and examples should now be better organized in the Project Explorer panel.

Release Notes
=============

v 4.0.0 (February 2013)
-----------------------

### Release Notes ###

New functionalities contained in the TD RF Module SDK Tools 4.0.0 include:

  - Documentation removed as it is now distributed through Github at <https://github.com/Telecom-Design/Documentation_TD_RF_Module>
  - Sources removed as they are now distributed through Github at <https://github.com/Telecom-Design/TD_RF_Module_SDK>