delete(instrfindall);

clear all; clc; close all;


% Initialize serial port
s = serial('COM5','BaudRate',115200);   
fopen(s);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fwrite(s,'1');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% variables
numberOfData = 500;
data = zeros(1, numberOfData);
i = 1;

% Main graph figure
figure(1);

hold on;
grid on;
%axis auto;
title('Incomming Data from External Device');
xlabel('Time');
ylabel('data');
xlim([0 numberOfData]);

% Start asynchronous reading
% readasync(s);

while(i<=numberOfData)  

      % Get the data from the serial object
      data(i) = fscanf(s, '%f');
      % Plot the data
     plot(i,data(i),'r*'); 
      % Draw and flush
      drawnow;
      %Increment the counter
      i=i+1;
      %pause(1);
end

% Give the external device some time…
pause(3);


%%%%%%%%%%%%%%%%%%%%%%



% Some of these crash the program – it depends. The serial port is left
% open, which is not good.                                              
% stopasync(s);
fclose(s); % bad
delete(s);
clear s;


