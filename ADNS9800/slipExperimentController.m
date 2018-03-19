%--------------------------------------------------------------------------
% NATIONAL UNIVERSITY OF SINGAPORE - NUS
% SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
% Singapore
%--------------------------------------------------------------------------
% Author: Andrei Nakagawa-Silva
% Contact: nakagawa.andrei@gmail.com
% URL: http://www.sinapseinstitute.org/
%--------------------------------------------------------------------------
% Description: This script receives data from the ADNS-9800 laser motion
% sensor and a FSR being collected by the NXP LPC1768 mbed board. A
% dual-type controller has been implemented. First, a proportional
% controller (force-based) is used to close the gripper until a minimum
% amount of force is applied indicating contact with the object. After
% that, a monotonic proportional-integral controller takes place. The
% ADNS9800 measures slip velocity and the controller responds accordingly.
% Had to adapt some things from Anna's approach so the output of the
% controller gives the position of the gripper rather than the amount of
% force that should be applied to the object.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%WIDOW-X
%--------------------------------------------------------------------------
%Serial port definitions
portName = 'COM3'; %port name
baud = 9600; %baudrate
%Object for handling serial communication
serialArbotix = serial(portName); 
%Configures the baudrate
set(serialArbotix,'BaudRate',baud);

%open the com port
fopen(serialArbotix);

%waits until the board is ready to receive commands
pause(2);

%SET POSITION
%write a package for setting a new position
%controlling servo #6: gripper
fwrite(serialArbotix,36); %header
fwrite(serialArbotix,0); %action
fwrite(serialArbotix,6); %servo
fwrite(serialArbotix,0); %position MSB
fwrite(serialArbotix,0); %position LSB
fwrite(serialArbotix,33); %end of package

%SET POSITION
%write a package for setting a new position
%controlling servo #6: gripper
pos = 607;
posMSB = uint8(bitshift(pos,-8)); %equivalent to pos>>8
posLSB = uint8(bitand(pos,255)); %equivalent to 0xFF
fwrite(serialArbotix,36); %header
fwrite(serialArbotix,0); %action
fwrite(serialArbotix,6); %servo
fwrite(serialArbotix,posMSB); %position MSB
fwrite(serialArbotix,posLSB); %position LSB
fwrite(serialArbotix,33); %end of package
% 
%SET POSITION
%write a package for setting a new position
%controlling servo #4: elbow
pos = 2640;
posMSB = uint8(bitshift(pos,-8)); %equivalent to pos>>8
posLSB = uint8(bitand(pos,255)); %equivalent to 0xFF
fwrite(serialArbotix,36); %header
fwrite(serialArbotix,0); %action
fwrite(serialArbotix,4); %servo
fwrite(serialArbotix,posMSB); %position MSB
fwrite(serialArbotix,posLSB); %position LSB
fwrite(serialArbotix,33); %end of package
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Parameters
dt=0.002; %sampling period
counter = 0; %counter for tracking the number of received samples
gain = 100; %gain factor to scale amplitude
x = 0; %initial value for motion in x
y = 0; %initial value for motion in y
adcForce = []; %store force values read from FSR sensor
xarr = []; %store all the x values
yarr = []; %store all the y values
deltax = []; %store the relative movement in x - velocity
deltay = []; %store the relative movement in y - velocity
maxTime = 20; %time duration in secods
maxSamples = floor(maxTime * (1.0/dt)); %total number of samples to be read
%creating a time vector
time = 1:maxSamples;
time = time.*dt;
counterFilter = 0; %counter for filter processing 
windowSize = 25; %size of the window --> 50 ms window at 500 Hz
filtForce = []; %array to store force values
posx = []; %array to store raw position in x
posy = []; %array to store raw position in x
px0=0; py0=0; %initial conditions for integrating velocity in x and y
fvx = []; fvy = []; fpx = []; fpy = []; %stores filtered signals
fpx0 = 0; fpy0 = 0; %initial conditions for integrating filtered signals
%--------------------------------------------------------------------------
%CONTROLLER PARAMETERS
%--------------------------------------------------------------------------
%determines which controller should be used
%0: P: proportional controller to initially close the gripper, based on
%minimum force threshold
%1: MPI controller: based on slip velocity
flagController = 0;
%determines initial gripper position according to the initialization of the
%WidowX
gripP0 = 512;
gripPos = []; %stores the gripper positions during the experiment
%initial output value desired on the FSR sensor: desired initial force
psetpoint = 600; %set-point for proportional controller
Kpp = 0.02; %proportional gain for the Pcontroller
%minimum error accepted for changing controllers
%if(psetpoint - FSRvalue <= 50), change controllers
minError = 50; 
Kp = 1; %proportional gain for the MPI controller
Ki = 1; %integral gain for the MPI controller
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
filename = input('When ready, type name of the file and press ENTER: ','s');
pause(6);
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Serial parameters
s = serial('COM6','BaudRate',115200); %creates object to handle serial
fopen(s); %open connection
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%ACQUISITION
%--------------------------------------------------------------------------
%while the desired number of samples have not been read, keep looping
while(counter < maxSamples)
    data = fread(s,1); %reads one byte
    if(data(1) == 36) %checks if it is the header, if positive, continue
        data = fread(s,6); %reads two bytes
        %mount the correct 16bit adc value
        %reads one byte
        datahead = fread(s,1);
        if(datahead(1) == 33) %checks if it is end of package            
            %if it is a valid package, then
            %force value
            force = (bitshift(data(1),8) + data(2));
            %store in array
            adcForce = [adcForce force];
            %change in x
            deltax = [deltax convTwosComp((bitshift(data(3),8) + data(4)))];
            %change in y
            deltay = [deltay convTwosComp((bitshift(data(5),8) + data(6)))];  
            %increase sample counter
            counter = counter+1;
            %integrate in x
            posx = [posx px0+deltax(counter)];
            %integrate in y
            posy = [posy py0+deltay(counter)];
            %updates the initial variables, necessary for integration
            px0 = posx(counter); %x
            py0 = posy(counter); %y         
            disp(['pkg ok!  force: ', num2str(force)]); %package ok
        end        
    end
    %every time new samples are read, increment the sample counter that
    %will coordinate when the signals will be filtered and used for giving
    %actions to the robot
    counterFilter = counterFilter + 1;
    %low-pass filters
    %controller actions
    if(counterFilter == windowSize)
        %mean value of the force signal
        filtForce = [filtForce mean(adcForce(counter-windowSize+1:counter))];
        %take the derivative of the previous position signal to form the
        %new velocity signal in x
        fvx = [fvx ((posx(counter)-posx(counter-windowSize+1))/(dt*windowSize))];
        %take the derivative of the previous position signal to form the
        %new velocity signal in y
        fvy = [fvy ((posy(counter)-posy(counter-windowSize+1))/(dt*windowSize))];
        %if it is the proportional controller, then changes in position
        %should be disconsidered, thus fpx and fpy = 0
        if(flagController == 0)
            fpx = [fpx 0];
            fpy = [fpy 0];
        else
            %if the slip controller is activated, then the integral of the
            %velocities are relevant, thus an integral is calculated
            if(flagController == 1)
                fpx = [fpx (fpx0+(dt*windowSize*fvx(end)))];
                fpy = [fpy (fpy0+(dt*windowSize*fvy(end)))];            
            end
        end
        %updates the initial conditions
        fpx0 = fpx(end);
        fpy0 = fpy(end);
        %zero the sample counter for the filtering process
        counterFilter = 0;
        
        %flagController = 0 --> Proportional Controller: force-based
        if(flagController == 0)
            %measures the error: setpoint - currentForce
            error = psetpoint - filtForce(end);
            %criteria for changing controller
            %if error is less than the tolerance, then change the
            %controller
            if(abs(error)<minError)
                %change the controller -> slip control
                flagController = 1;
                %saves the index where the controller was changed
                idxChangeController = length(gripPos)+1;
            else
                %otherwise, Pcontroller gives commands
                %the controller output is given by
                %gripP0 - Kp*error where gripP0 is the last position of the
                %gripper. Therefore, in every iteration, the gripper will
                %close an amount proportional to the error. The subtraction
                %is due to 512: gripper completely opened and 0: gripper
                %fully closed.
                pcontrol = gripP0 - Kpp*error;
                %floor to round down values back to integer
                pcontrol = floor(pcontrol);
                %saturation: if greater than maximum possible position,
                %output is equal to maximum
                if(pcontrol > 512)
                    pcontrol = 512;
                end
                %saturation: if less than minimum possible position, output
                %is equal to minimum
                if(pcontrol < 0)
                    pcontrol = 0;
                end               
                %retrieve the MSB part of the gripper position
                posMSB = uint8(bitshift(pcontrol,-8)); %equivalent to pos>>8
                %retrieve the LSB part of the gripper position
                posLSB = uint8(bitand(pcontrol,255)); %equivalent to 0xFF
                %sends the new position to WidowX
                fwrite(serialArbotix,36); %header
                fwrite(serialArbotix,0); %action
                fwrite(serialArbotix,6); %servo: gripper
                fwrite(serialArbotix,posMSB); %position MSB
                fwrite(serialArbotix,posLSB); %position LSB
                fwrite(serialArbotix,33); %end of package
                %updates griP0 to the current position
                gripP0 = pcontrol;
            end
            %saves the new gripper position
            gripPos = [gripPos pcontrol];
        else
            %flagController = 1 --> MPI Controller: slip velocity based
            if(flagController == 1)
                %just to keep the same nomenclature from the simulink
                %project created by Anna
                %current velocity
                v = fvy(end);
                %current position
                z = fpy(end);
                %PI control-law
                mpiCont = Kp*sign(v)*abs(v) + Ki*sign(z)*abs(z);
                %Saves the new grip position: uses floor to round down to
                %integer the position
                gripPos = [gripPos floor(gripP0-mpiCont)];
                %saturation: if greater than maximum possible position,
                %output is equal to maximum
                if(gripPos(end) > 512)
                    gripPos(end) = 512;
                end
                %saturation: if less than minimum possible position,
                %output is equal to minimum
                if(gripPos(end) < 0)
                    gripPos(end) = 0;
                end
                %Monotonic aspect of the controller
                %If the current position is greater than the previous one,
                %then replace it with the minor value. This indicates that
                %force will only increase when using the MPI controller,
                %which makes sense when considering that in order to
                %suppress slip, grip force should increase and be held in a
                %greater value.
                if(gripPos(end) > gripPos(end-1))
                    gripPos(end) = gripPos(end-1);
                end
                %retrieve the MSB part of the gripper position
                posMSB = uint8(bitshift(gripPos(end),-8)); %equivalent to pos>>8
                %retrieve the LSB part of the gripper position
                posLSB = uint8(bitand(gripPos(end),255)); %equivalent to 0xFF
                %sends the new position to WidowX
                fwrite(serialArbotix,36); %header
                fwrite(serialArbotix,0); %action
                fwrite(serialArbotix,6); %servo: gripper
                fwrite(serialArbotix,posMSB); %position MSB
                fwrite(serialArbotix,posLSB); %position LSB
                fwrite(serialArbotix,33); %end of package
            end
        end        
    end
end
%closes the serial port communication with the mbed LPC1768
fclose(s);
%closes the serial port communication with the WidowX-ArbotiX
fclose(serialArbotix);
%time vector for the filtered signals
filtTime = 1:length(fvx);
filtTime = filtTime .* (1.0/(dt*windowSize));
%--------------------------------------------------------------------------
input('When ready, press ENTER to finish: ','s');
%--------------------------------------------------------------------------
%save all data in a .mat file
resp.time = time;
resp.rawDeltax = deltax;
resp.rawDeltay = deltay;
resp.rawDistx = posx;
resp.rawDisty = posy;
resp.adcForce = adcForce;
resp.filtTime = filtTime;
resp.filtForce = filtForce;
resp.filtVelx = fvx;
resp.filtVely = fvy;
resp.filtPosx = fpx;
resp.filtPosy = fpy;
resp.gripperPos = gripPos;
save(['experimentControl_',filename],'-struct','resp');