% RPi TCP Server Testing
% REQUIRES MATLAB 2024a and the "connectionCB.m" MATLAB File

% Check if the "NASA" TCP server exists
if isempty(tcpserverfind(Tag="NASA")) == 1
    % get rid of all former variables & start with a clean workspace
    clear all
    disp("Creating new server")
    % Once the server has been initialized, it will continue running even
    % after the MATLAB script has finished

    IP = "169.254.186.240"; % This number can be obtained by running the
    % "ipconfig" command from Windows cmd or "ifconfig" from a linux terminal
    
    PORT = 5000;    %This number is semi-arbitrary, but should be set higher
    % than 1000 to avoid conflicts with other ports on the computer

    server = tcpserver(IP, PORT,"ConnectionChangedFcn",@connectionCB,Tag="NASA") % Initialize the tcp server
    % configureCallback(server, "terminator", readFcn)
    disp("New server created")
end

input("Press Enter when ready to proceed")

% Initialize the array of data
data = [0 1 2 3];
i=0;
time = zeros(1,252);
overall = tic;
while true
    i = i+1;
    % Get the length of the array you intend to send
    messageSize = size(data); % excludes the index of the message size

    % Restructure the 'message' to include the expected size
    tcp_message = [messageSize(1,2) data];
    single = tic;
    write(server, tcp_message);
    data(1,1:4) = data(1,1:4)+1;
    data = [data i];
    
    while(server.NumBytesAvailable == 0)
        % Wait for bytes to become available in the server buffer
    end

    reply = read(server, server.NumBytesAvailable);
    reply = typecast(uint8(reply), 'uint8');
    time(i) = toc(single);
    if tcp_message(1) == 255
        disp("Iteration complete")
        break
    end
%     pause(0.1)
end

totalTime = toc(overall)
avgTime = mean(time)

%%%%%% Results with no print statements (MATLAB or Python): %%%%%%
% Total run time: 4.1-4.2s
% Avg communication time: 0.016-0.0165s

N = 'no';
Y = 'yes';
closeServer = input("Would you like to close the server (Y/N)?");

if size(closeServer) == 3
    delete(server)
    clear server
    disp("Server has been closed")
else
    disp("Server is still active")
end