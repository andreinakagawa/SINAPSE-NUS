%--------------------------------------------------------------------------
% NATIONAL UNIVERSITY OF SINGAPORE - NUS
% SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
% Singapore
%--------------------------------------------------------------------------
% Author: Andrei Nakagawa-Silva
% Contact: nakagawa.andrei@gmail.com
%--------------------------------------------------------------------------
% Description: This script implements a Proportional Controller with
% force feedback for automatically closing the gripper. The set-point
% is the desired force. 
% Experimental setup: One Arduino Uno board with a Force Resistive Sensor
% (FSR) connected to an Analog Input (A0) and One WidowX robot. This Matlab
% program receives data from Arduino Uno via serial and finds the next
% position of the gripper based on the error (P-control). Then, it sends
% the new position to WidowX via serial as well.
% The sampling frequency of the Arduino Uno was set to 100 Hz. The code
% is available in this repository as well in the folder:
% "Arduino/proportional_controller".
% The sampling frequency of the ADC determines the speed of the controller
%--------------------------------------------------------------------------
% Hint: If an error occurs and it is not possible to open communication
% again, just run this command: fclose(instrfind())
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%ArbotiX Serial Communication
serialArbotix = serial('COM3','BaudRate',38400);
fopen(serialArbotix);
%Arduino Serial Communication
serialArduino = serial('COM5','BaudRate',9600);
fopen(serialArduino);
flushinput(serialArduino)
%--------------------------------------------------------------------------
pause(2); %waits for the system to be fully operational
%--------------------------------------------------------------------------
%SET POSITION
%this initial command serves as synchronization signal: can be solved
%by implementing a handshake protocol in WidowX.
%write a package for setting a new position
%controlling servo #6: gripper
fwrite(serialArbotix,36); %header
fwrite(serialArbotix,0); %action
fwrite(serialArbotix,6); %servo
fwrite(serialArbotix,0); %position MSB
fwrite(serialArbotix,0); %position LSB
fwrite(serialArbotix,33); %end of package
%--------------------------------------------------------------------------
%SETS THE GRIPPER TO BE FULLY OPENED IN THE BEGINNING
%write a package for setting a new position
%controlling servo #6: gripper
pos = 512;
posMSB = uint8(bitshift(pos,-8)); %equivalent to pos>>8
posLSB = uint8(bitand(pos,255)); %equivalent to 0xFF
fwrite(serialArbotix,36); %header
fwrite(serialArbotix,0); %action
fwrite(serialArbotix,6); %servo
fwrite(serialArbotix,posMSB); %position MSB
fwrite(serialArbotix,posLSB); %position LSB
fwrite(serialArbotix,33); %end of package
%--------------------------------------------------------------------------
% CONTROLLER PARAMETERS
Kp = 0.1; %proportional constant
setPoint = 25; %the desired force
p0 = 512; %initial position
prevPos = p0; %previous position
force = 0; %force read from Arduino's ADC
%control-loop
while(true)
    %data acquisition
    %read the incoming package from Arduino 
    %package protocol: [header=36][dataMSB][dataLSB][end=33]
    data = fread(serialArduino,1); %reads one byte
    if(data(1) == 36) %checks if it is the header, if positive, continue
        data = fread(serialArduino,2); %reads two bytes
        %mount the correct 16bit adc value
        force = (bitshift(data(1),8) + data(2)); 
        %reads one byte
        data = fread(serialArduino,1);
        if(data(1) == 33) %checks if it is end of package
            disp(['pkg ok!  force: ', num2str(force)]); %package ok
        end
    end
    
    %after reading the sensed force, the control-loop should kick-in
    error = setPoint - force; %error estimate
    %stop criteria or error tolerance +- 5 in adc reading
    if(abs(error) > 5)
        %finds the new position
        %Proportional Control: u(t) = u(t-1) - Kp*e(t)
        %A minus operation takes place since the control signal
        %needs to decrease so the gripper can be closed
        newPos = prevPos - Kp*error;
        %uses floor to find the new position
        %range: 0-512 uint8 integers
        newPos = floor(newPos);
        posMSB = uint8(bitshift(newPos,-8)); %equivalent to pos>>8
        posLSB = uint8(bitand(newPos,255)); %equivalent to 0xFF
        %sends the new position to WidowX
        fwrite(serialArbotix,36); %header
        fwrite(serialArbotix,0); %action
        fwrite(serialArbotix,6); %servo
        fwrite(serialArbotix,posMSB); %position MSB
        fwrite(serialArbotix,posLSB); %position LSB
        fwrite(serialArbotix,33); %end of package
        %updates previous position
        prevPos = newPos;
        %displays current position in command window
        disp(['gripper position: ', num2str(newPos)]);
    else %if stopping criteria is met, breaks the control loop
        break;        
    end    
end
%--------------------------------------------------------------------------
%Closes communication with Arduino Uno and WidowX.
fclose(serialArbotix);
fclose(serialArduino);
%--------------------------------------------------------------------------