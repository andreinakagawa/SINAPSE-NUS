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
% sensor and a FSR being collected by the NXP LPC1768 mbed board. Then,
% data is filtered using a 50 ms window to check the output and possible
% behavior of the complete system at this frame rate. Data is acquired at
% 500 Hz, but actions take place every 50 ms.
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
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
filename = input('When ready, type name of the file and press ENTER: ','s');
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Serial parameters
s = serial('COM6','BaudRate',115200); %creates object to handle serial
fopen(s); %open connection
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%ACQUISITION
%--------------------------------------------------------------------------
counterFilter = 0; %counter for filter processing 
windowSize = 25; %size of the window --> 50 ms window at 500 Hz
filtForce = []; %array to store force values
posx = []; %array to store raw position in x
posy = []; %array to store raw position in x
px0=0; py0=0; %initial conditions for integrating velocity in x and y
fvx = []; fvy = []; fpx = []; fpy = []; %stores filtered signals
fpx0 = 0; fpy0 = 0; %initial conditions for integrating filtered signals
%while the desired number of samples have not been read, keep looping
while(counter < maxSamples)
    data = fread(s,1); %reads one byte
    if(data(1) == 36) %checks if it is the header, if positive, continue
        data = fread(s,6); %reads two bytes        
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
    if(counterFilter == windowSize)
        %mean value of the force signal
        filtForce = [filtForce mean(adcForce(counter-windowSize+1:counter))];
        %derivative of the integrated signal in x
        fvx = [fvx ((posx(counter)-posx(counter-windowSize+1))/(dt*windowSize))];
        %derivative of the integrated signal in y
        fvy = [fvy ((posy(counter)-posy(counter-windowSize+1))/(dt*windowSize))];
        %filtered position in x
        fpx = [fpx (fpx0+(dt*windowSize*fvx(end)))];
        %filtered position in y
        fpy = [fpy (fpy0+(dt*windowSize*fvy(end)))];            
        %updates initial conditions for the integration
        fpx0 = fpx(end);
        fpy0 = fpy(end);
        %zeros the sample counter for filter processing
        counterFilter = 0;
    end
end
%closes the serial port communication
fclose(s);
%--------------------------------------------------------------------------
input('When ready, press ENTER to finish: ','s');
%--------------------------------------------------------------------------
%save data in a .mat file
resp.time = time;
resp.deltax = deltax;
resp.deltay = deltay;
resp.adcForce = adcForce;
save(['experiment_',filename],'-struct','resp');