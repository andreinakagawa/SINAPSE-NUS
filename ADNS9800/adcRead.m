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
% sensor and a FSR being collected by the NXP LPC1768 mbed board.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Parameters
dt=0.001; %sampling period
counter = 0; %counter for tracking the number of received samples
gain = 100; %gain factor to scale amplitude
x = 0; %initial value for motion in x
y = 0; %initial value for motion in y
adcForce = []; %store force values read from FSR sensor
xarr = []; %store all the x values
yarr = []; %store all the y values
xdelta = []; %store the relative movement in x - velocity
ydelta = []; %store the relative movement in y - velocity
maxTime = 1; %time duration in secods
maxSamples = floor(maxTime * (1.0/dt)); %total number of samples to be read
%creating a time vector
time = 1:maxSamples;
time = time.*dt;
%--------------------------------------------------------------------------
%Serial parameters
s = serial('COM8','BaudRate',115200); %creates object to handle serial
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
        force = (bitshift(data(1),8) + data(2));
        deltax = convTwosComp((bitshift(data(3),8) + data(4)));
        deltay = convTwosComp((bitshift(data(5),8) + data(6)));
        %reads one byte
        data = fread(s,1);
        if(data(1) == 33) %checks if it is end of package
            disp(['pkg ok!  force: ', num2str(deltax)]); %package ok
            counter = counter+1;
        end
    end
    disp(num2str(force));
end
fclose(s);
