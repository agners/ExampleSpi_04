This example attend an Interrupt at pin SODIM  and at Falling Edge Toggle the led connect to the pin SODIM 98.
Also we implement the RPMSG inside the same example in a different task.

There is also tasks running and semaphore signaling the IRQ event

- Download the Linaro provided ARM Embedded toolchains:
	https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update

- Extract the toolchain:
tar xjf ~/Downloads/gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2

- Export the toolchain path according to the place you has extracted the file:

export ARMGCC_DIR=/home/prjs/colibri_iMX7/m4/gcc-arm-none-eabi-4_9-2015q3

- Edit the Project Name:

vi src/CMakeLists.txt

-SET(Project_Name Project_Base_M4)
+SET(Project_Name ProjectNEW)

- Create eclipse .cproject and .project file:

cd Project_Base_M4
./armgcc/build.sh

- Import project Eclipse

Using eclipse import project find the folder armgcc


