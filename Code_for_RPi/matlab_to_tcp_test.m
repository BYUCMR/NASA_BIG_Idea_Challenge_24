% Code for testing rpi tcp implementation
% Before running this file for the first time of the day, enter
% "newFile = True"
% in the Command Window. Not needed for tests that happen after the first
% unless you wish to restart the TCP Socket, Port, and Server
N = 'no';
Y = 'yes';

% Check if the file is being run for the first time
if (newFile == true)
    % get rid of all former variables & start with a clean workspace
    clear all
    "Creating new server"
    % Once the server has been initialized, it will continue running even
    % after the MATLAB script has finished
    server = tcpserver("169.254.186.240", 5000); % Ethernet ip address
    "New server created"
    newFile = false;
    % Flip the boolean to prevent the server from getting re-created on every run
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
        exitmsg = "Iteration complete"
        break
    end
%     pause(0.1)
end

totalTime = toc(overall)
avgTime = mean(time)

%%%%%% No print statements (MATLAB or Python): %%%%%%
% Total run time: 4.1-4.2s
% Avg communication time: 0.016-0.0165s

closeServer = input("Would you like to close the server (Y/N)?");

if size(closeServer) == 3
    delete(server)
    clear server
    newFile = true;
    "Server has been closed"
else
    "Server is still active"
end