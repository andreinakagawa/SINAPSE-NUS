%--------------------------------------------------------------------------
% NATIONAL UNIVERSITY OF SINGAPORE - NUS
% SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
% Singapore
%--------------------------------------------------------------------------
% Author: Andrei Nakagawa-Silva
% Contact: nakagawa.andrei@gmail.com
%--------------------------------------------------------------------------
% Description: This script can be used for testing communication with the
% ArbotiX platform and controlling WidowX.
%--------------------------------------------------------------------------
% Hint: If an error occurs and it is not possible to open communication
% again, just run this command: fclose(instrfind())
%--------------------------------------------------------------------------
%Serial port definitions
portName = 'COM3'; %port name
baud = 9600; %baudrate
%Object for handling serial communication
serialRobot = serial(portName); 
%Configures the baudrate
set(serialRobot,'BaudRate',baud);

%open the com port
fopen(serialRobot);

%waits until the board is ready to receive commands
pause(10);

%SET POSITION
%write a package for setting a new position
fwrite(serialRobot,36); %header
fwrite(serialRobot,0); %action
fwrite(serialRobot,6); %servo
fwrite(serialRobot,0); %position MSB
fwrite(serialRobot,0); %position LSB
fwrite(serialRobot,33); %end of package

%waits for sending a new command
pause(1);

%GET POSITION
%write a package for receiving the position of the servo
fwrite(serialRobot,36); %header
fwrite(serialRobot,1); %action
fwrite(serialRobot,6); %servo
fwrite(serialRobot,0); %position MSB - don't care
fwrite(serialRobot,0); %position LSB - don't care
fwrite(serialRobot,33); %end of package

%waits for reading the serial buffer
pause(1);

%retrieves data
%package sent from ArbotiX is 5 bytes long
data = fread(serialRobot,5);

%Closes communication
fclose(serialRobot);
%--------------------------------------------------------------------------