function reply = readFcn(src,~)
reply = read(server, server.NumBytesAvailable);
reply = typecast(uint8(reply), 'uint8');