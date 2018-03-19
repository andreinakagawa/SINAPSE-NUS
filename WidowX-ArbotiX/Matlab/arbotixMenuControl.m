%--------------------------------------------------------------------------
% NATIONAL UNIVERSITY OF SINGAPORE - NUS
% SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
% Singapore
%--------------------------------------------------------------------------
% Author: Andrei Nakagawa-Silva
% Contact: nakagawa.andrei@gmail.com
%--------------------------------------------------------------------------
% Description: This script can be used for sending commands to the WidowX
% and receiving position from the servos so it is easier to setup new poses
% as desired.
%--------------------------------------------------------------------------
%Serial communication parameters
serialRobot = serial('COM3','BaudRate',9600);
fopen(serialRobot);
%Pause before starting
pause(6);

%MENU
while(true)
    %Options: Set position or read position
    option = input('Choose: 1: Set position, 2: Read pose, 3: Exit : ');
    
    %SET POSITION
    if(option == 1)        
        %write a package for setting a new position
        %which servo
        servoId = input('Choose a servo: ');
        if(servoId < 1 || servoId > 6)
            disp('Wrong id');
            continue;        
        end
        %which position
        pos = input('Choose a position: ');
        %if(pos < 0 || pos > 1023)
        %    disp('out of range');
        %    continue;
        %end
        %Sends the packet for controlling a position
        posMSB = uint8(bitshift(pos,-8)); %equivalent to pos>>8
        posLSB = uint8(bitand(pos,255)); %equivalent to 0xFF
        fwrite(serialRobot,36); %header
        fwrite(serialRobot,0); %action
        fwrite(serialRobot,servoId); %servo
        fwrite(serialRobot,posMSB); %position MSB
        fwrite(serialRobot,posLSB); %position LSB
        fwrite(serialRobot,33); %end of package
    else
        %GET POSITION
        if(option == 2)
            %which servo
            servoId = input('Choose a servo: ');
            if(servoId < 1 || servoId > 6)
                disp('Wrong id');
                continue;        
            end
            %write a package for receiving the position of the servo        
            fwrite(serialRobot,36); %header
            fwrite(serialRobot,1); %action
            fwrite(serialRobot,servoId); %servo
            fwrite(serialRobot,0); %position MSB - don't care
            fwrite(serialRobot,0); %position LSB - don't care
            fwrite(serialRobot,33); %end of package

            %waits for reading the serial buffer
            pause(1);

            %retrieves data
            %package sent from ArbotiX is 5 bytes long
            data = fread(serialRobot,5);
            %retrieves the position by combining MSB and LSB
            position = bitshift(data(3),8) + data(4)
        else
            if(option == 3)
                break;
            end
        end
    end
end
%closes serial communication
fclose(serialRobot);
%--------------------------------------------------------------------------
