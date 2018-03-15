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
% sensor being collected by the NXP LPC1768 mbed board.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Parameters
dt=0.001; %sampling period
counter = 0; %counter for tracking the number of received samples
gain = 100; %gain factor to scale amplitude
x = 0; %initial value for motion in x
y = 0; %initial value for motion in y
xarr = []; %store all the x values
yarr = []; %store all the y values
xdelta = []; %store the relative movement in x - velocity
ydelta = []; %store the relative movement in y - velocity
maxTime = 20; %time duration in secods
maxSamples = floor(maxTime * (1.0/dt)); %total number of samples to be read
%creating a time vector
time = 1:maxSamples;
time = time.*dt;
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
    datax = fgetl(s); %reads a line from the serial buffer
    datay = fgetl(s); %reads a line from the serial buffer
    %integrate to get distance in x
    x = x + (str2double(datax)*gain*dt);
    %integrate to get distance in y
    y = y + (str2double(datay)*gain*dt);
    %save x and y values
    xarr = [xarr x];
    yarr = [yarr y];
    %save relative movement in x and y
    xdelta = [xdelta str2double(datax)];
    ydelta = [ydelta str2double(datay)];
    %print on screen the x and y integrated values
    disp(['x: ', num2str(x),' | y: ',num2str(y)]);
    %increase sample counter
    counter = counter+1;
end    
fclose(s); %closes serial port
%--------------------------------------------------------------------------
%Plots motion in Y-axis
%time vector
figure();
subplot(2,1,1);
plot(time, ydelta);
ylabel('Velocity in Y');
title('ADNS9800 Motion Sensor Output');
subplot(2,1,2);
plot(time, yarr);
ylabel('Distance in Y');
xlabel('Time (s)');
%--------------------------------------------------------------------------
%derivative over a window
%look between last and first sample in a window to determine velocity,
%instead of looking sample by sample
%applying only to motion in Y -> interesting for slip experiments
windowSize = 50; %size of the window in number of samples
newSignal = [0]; %stores the new velocity signal
%Loop through all the signal in windows of size 'windowSize'
for k=1:windowSize:length(yarr)
    %if there is not enough samples from current index to the end of the
    %vector, then breaks the loop
    if(k+windowSize > length(yarr))
        break;
    end
    %the new velocity signal is taken by the change in the integrated
    %signal (yarr) from time(k) to time(k+windowSize)
    deriv = (yarr(k+windowSize)-yarr(k))/(dt*windowSize);
    %stores the new signal
    newSignal = [newSignal deriv];
end
%plots the new signal
newTime = 1:length(newSignal);
newTime = newTime .* (dt*windowSize);
figure();
plot(newTime,newSignal);
title('Corrected Velocity Signal');
xlabel('Time (s)');
ylabel('Velocity');
%--------------------------------------------------------------------------
%Saves data in a .mat file
resp.time = time;
resp.integX = xarr;
resp.integY = yarr;
resp.deltaX = xdelta;
resp.deltaY = ydelta;
resp.filtTime = newTime;
resp.filtY = newSignal;
save('expResults','-struct','resp');
%--------------------------------------------------------------------------