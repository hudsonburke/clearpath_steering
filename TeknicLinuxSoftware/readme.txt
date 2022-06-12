

                          Teknic sFoundation for Linux
                                   2021-04-12


                                 Setup Overview
To start using sFoundation, you'll have to complete the following steps:
 1. Configure permissions for interacting with serial ports
 2. Build the sFoundation shared library
 3. Install the sFoundation shared library

This document will also cover how to...
 4. Install the SC4-Hub driver if you plan on connecting it via USB
 5. Build and run the example projects
 6. Compile your own application
 7. Compile sFoundation for Single Board Computers


                                       0
                              Before You Start...
Make sure you have the required software:
 - tar (to extract package contents)
    tested: 1.32, 1.3
 - make (to build sFoundation)
    tested: 4.3, 4.2.1
 - g++ (to build sFoundation; must support C++ 11)
    tested: 10.2.1, 9.3.0, 8.3.0
 - ldconfig (for installing sFoundation)
    tested: 2.32, 2.31, 2.28

Make sure you have permission:
 - To install sFoundation shared library for system use, you will need write
   access to /usr/local/lib
 - To install the SC4-Hub USB driver, you will need root access

These instructions have been verified to work exactly as written in
 - Ubuntu 20.04 (x64)
 - Fedora Workstation 33 (x64)
 - Debian 10.9 (x64)
 - Raspberry Pi OS (buster) (arm)
 - Ubuntu 18.04 on BeagleBone Black (arm) (linux 4.19.94-ti-r36)

 > Note: Don't lose hope if your distribution isn't listed; there's a good
   chance you will be able to get everything working with minimal tinkering


                                       1
                            Serial Port Permissions
In most distributions, this will suffice:
 1. Add your user to the dialout group
    $ sudo usermod -aG dialout <your_username>
 2. Reboot or log out and back in for your new permissions to take effect
 3. Make sure your user is now in the dialout group
    $ groups
    yourUsername adm dialout sudo cdrom ...
                     ^^^^^^^


                                       2
                        Build sFoundation Shared Library
 1. Extract the sFoundation source bundled in this package. This will inflate
    into multiple directories
    $ tar -xvf sFoundation.tar
 2. Navigate to the sFoudnation directory
    $ cd sFoundation
 3. Run make. This will build libsFoundation20.so in the current directory
    $ make

    > Note: By default, the g++ -w flag is used to suppress warnings. If you
      want to see them, remove that flag from the CXXFLAGS variable in the
      makefile. Either way, sFoundation should build without error.


                                       3
                       Install sFoundation Shared Library
The sFoundation library can be installed for systemwide use by all users, or on
a single-user basis. You can pick one or the other, depending on your use-case
and permissions. You DO NOT need to do both.

Systemwide Install:
You will need permission to write to system directories (specifically,
/usr/local/lib). If you lack the required permissions, you may need to contact
your administrator, or consider using the library locally.
 1. Copy MNuserDriver.xml and libsFoundation20.so to /usr/local/lib
    $ sudo cp {MNuserDriver20.xml,libsFoundation20.so} /usr/local/lib
 2. Run ldconfig to set up the proper symbolic links and add the sFoundation
    library to the linker search path
    $ sudo ldconfig
 3. For sanity, verify that the system detected the new shared library
    $ ldconfig -p | grep "sFoundation"

    The expected output would be something like this:
    libsFoundation20.so.1 (libc6,x86-64) => /usr/local/lib/libsFoundation20.so.1

    > Note: ldconfig is typically in /usr/sbin/ldconfig. On some distributions,
      including Debian, /usr/sbin/ is not in the default search path for
      standard users. You might need to specify the full path.
      $ /usr/sbin/ldconfig -p | grep "sFoundation"

    > Note: If the above command has no output, ldconfig might not be configured
      to search /usr/local/lib by default. This seemed to be the case in Fedora
      33.
            1. Make sure /usr/local/lib is indeed absent from the ld search 
               path. You should get no output.
               $ grep -re "/usr/local/lib" /etc/ld.so.c*

               > Note: If you did get output, make sure the appropriate files
                 are included in /etc/ld.so.conf
            2. Make sure /usr/local/lib is explicitly listed or included in 
               /etc/ld.so.conf. You could create a new file in an included 
               directory under /etc/ld.so.conf.d/, or simply append to 
               /etc/ld.so.conf.
               $ echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
            3. Try running sudo ldconfig again, then ldconfig -p. You should get
               output this time.

Single-User Install:
You can point the linker and loader to your libsFoundation20.so wherever it was
built. This has the advantage that you don't need super user access, but comes
with the drawback that users may need to compile your application for
themselves, depending on where libsFoundation20.so is located.
 1. Now would be a good time to copy MNuserDriver.xml and libsFoundation20.so
    to wherever you want them to live. Keep in mind that moving the .so files
    will require you to recompile dependent executables. They can remain where
    they were built if you prefer.
    $ cp {MNuserDriver20.xml,libsFoundation20.so} /home/user/teknic/
 2. Create the necessary symbolic links to the sFoundation library (the soname).
    $ ldconfig -n /home/user/teknic/

    > Note: Replace the path with the directory you copied libsFoundation20.so
      to in step 1.
    > Note: ldconfig is typically in /usr/sbin/ldconfig. On some distributions,
      including Debian, /usr/sbin/ is not in the default search path for
      standard users. You might need to specify the full path.
      $ /usr/sbin/ldconfig -n /home/user/teknic/
 3. Make sure you compile your application with rpath. Section 6 contains 
    specifics about this.

Further Reading:
The Linux Documentation Project has a good explanation of how shared libraries
work, if you are having trouble getting things set up:
https://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html


                                       4
                               SC4-Hub USB Driver
This section can be skipped if you won't be connecting the SC4-Hub to your PC
via USB. 
 1. Navigate back to the root directory of this package, if needed. If you're
    coming from Section 2/3:
    $ cd ..
 2. Extract the driver package. This will create a directory called
    ExarKernelDriver.
    $ tar -xvf Teknic_SC4Hub_USB_Driver.tar
 3. View the installation instructions in
    ExarKernelDriver/driver_readme.txt in a text editor.



                                       5
                                Example Projects
There are several examples that demonstrate how to interact with the sFoundation
library. To build them, simply change to the example directory and run make.
For example, to run HelloWorld:
 1. cd SDK_Examples/HelloWorld
 2. make
 3. ./HelloWorld

 > Note: If sFoundation is installed in a non-standard location (outside
   /usr/local/lib, or /usr/lib), point g++ to it, and embed the path of
   the shared library into the executable using the rpath linker option. The
   Makefiles allow you to do this by defining RPATH on the command line. For
   example, if libsFoundation20.so is located in /home/user/stuff, your make
   command (step 2 above) would look like this:
 $ make RPATH=/home/user/stuff


                                       6
                         Compiling Your Own Application
The example makefiles may be helpful as a reference to help you set up your
target build system. Here is a summary of how the library can be used in a
custom application:
 1. Add sFoundation headers, located at inc/inc-pub, to the compiler's include
    path (-I in g++).
 2. Tell the linker to link against the sFoundation20 and pthread libraries
    (-lsFoundation20 and -lpthread respectively in g++)

If you did not install libsFoundation20 to a directory in the linker path (i.e.
outside /usr/local/lib, /usr/lib, etc.), you will need to pass additional flags
to g++. For reference, see the Example project Makefiles, which allow you to
link to a nonstandard location using RPATH (Section 5).
 3. Specify the -L<dir> option. For example, if libsFoundation20.so is in 
	/home/user/sf, simply add -L/home/user/sf to your build command.
 4. Pass the rpath option to the linker so that the shared library
    loader knows which directory to load sFoundation from at runtime. Include
	-Wl,-rpath=/home/user/sf in your build command if libsFoundation20.so is
	located in /home/user/sf.


                                       7
                     sFoundation on Single Board Computers
TODO DCB Talk about how this used to work in eclipse...
All of the information in this document should apply to compiling natively on
BeagleBone and Raspberry Pi. Cross-compilation for ARM on x86 is no longer
officially supported.
