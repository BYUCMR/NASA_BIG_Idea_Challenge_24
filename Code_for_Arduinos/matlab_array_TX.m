% Create a serialport object called serialportObj that connects to the port "COM3" with a
% default baud rate of 9600.
s = serialport("COM3",9600)

% Write the data "[-7 -7 14 0 -14 0 7 7]" as string using the serialport object
% serialportObj.
write(s,"[-7 -7 14 0 -14 0 7 7]","string");
x = s.NumBytesAvailable;
data1 = read(s,46,"string");





