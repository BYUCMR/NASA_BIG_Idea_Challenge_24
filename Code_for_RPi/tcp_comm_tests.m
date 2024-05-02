% Code for testing rpi tcp implementation
clear all
var = "Creating new server"
server = tcpserver("169.254.186.240", 5000)
var = "New server created"
% client = tcpclient("localhost", 5000)
% server.ClientAddress
% message = [0 1 2 3 4 5 6 7];
% write(client, message, "uint8");
while true
    message = read(server, 5)
    write(server, message);
    var = "Code completed"
end
