The EtherCAT application is expected to be used against the ESI xml file given in esi folder.If the application should work with another ESI xml file, user will need to generate a corresponding ESI header file ( tiesc_eeprom.h ) and re-build the ecat_appl with the generated .h file.

Steps for generating ESI header file is given below.
1. Generate the binary file equilant to ESI xml file. Pease see Generating EEPROM binary.
2. Convert the binary file to header file using the bin2header.exe utility. This utility can be found in  $(IA_SDK_HOME)\tools\bin2header 

Usage :
'bin2header.exe 'binary_filename' 'header_filename' 'out_array_name' 

Example: 
bin2header.exe "C:\Documents and Settings\user\Desktop\Box1.bin" "C:\Documents and Settings\user\Desktop\tiesc_eeprom.h" tiesc_eeprom

Replace the existing file with new header file ( tiesc_eeprom.h) to $(IA_SDK_HOME)\Protocols\ethercat_slave\ecat_appl\src
Rebuild the application.