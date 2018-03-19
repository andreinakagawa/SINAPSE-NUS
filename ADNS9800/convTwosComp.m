%--------------------------------------------------------------------------
% NATIONAL UNIVERSITY OF SINGAPORE - NUS
% SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
% Singapore
%--------------------------------------------------------------------------
% Author: Andrei Nakagawa-Silva
% Contact: nakagawa.andrei@gmail.com
% URL: http://www.sinapseinstitute.org/
%--------------------------------------------------------------------------
% Description: This function retrieves the twos complement value of the
% input. Necessary when using the ADNS9800 laser motion sensor for proper
% reading of teh sensor.
%--------------------------------------------------------------------------
function resp = convTwosComp(data)
    if(data >= 32768)                   
        resp = -1 * (bitxor(data,65535)+1);                
    else
        resp = data;
    end    
    %disp([num2str(data),' ',num2str(resp)]); %for debugging
end