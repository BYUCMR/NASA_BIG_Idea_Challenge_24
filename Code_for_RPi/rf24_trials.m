clear all
mypi = raspi
myRF24 = spidev(mypi, 'CE0', 0);
configurePin(mypi, 22, 'DigitalOutput');
writeDigitalPin(mypi, 22, 0);
pause(0.5);
writeDigitalPin(mypi, 22, 1);
writeDigitalPin(mypi, 22, 0);
out = writeRead(myRF24, [hex2dec('08') hex2dec('D4')], 'uint8')
out = dec2base(out(1))