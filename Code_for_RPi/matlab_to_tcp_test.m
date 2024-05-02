% Code for testing rpi tcp implementation
clear all
"Creating new server"
server = tcpserver("169.254.186.240", 5000) % Ethernet ip address connected to the pi
"New server created"
message = [0 1 2 3];
% write(client, message, "uint8");
pause(10);
while true
    % Can successfully send a message with multiple entries
    write(server, message);
    message = message + 1;
    % Replace the pause statement below with a 'while'
    % loop that checks to see if server.NumBytesAvailable is nonzero
    pause(0.1)
    reply = read(server, server.NumBytesAvailable);
    reply = char(reply)
        
    if strcmp(reply, 'fail')
        exitmsg = "Something failed"
        break
    elseif message(1) == 255
        exitmsg = "Iteration complete"
        break
    end
    pause(0.1)
end