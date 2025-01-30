Things to try:
- Code your own auto ack
- Do not put parent nodes on pipe 0. Put them elsewhere
- Look at the state of the board for debugging
- Try with auto ack and no inerrupts with updated base node
- Retry but with smalller BPS and less aggressive retries
- Broadcast the message to everyone
- Change power consumption level
- Run the code with DEBUG_RF24 defined

- It could possibly be how I am working with the addresses. The documentation says that only the LSB is different for the addresses between pipes 1-5. I pass in addresses that are unique in the last byte, allowing up to 121 unique addresses. But maybe there is something here that I am missing as well
  - However, the last byte of my addresses are based on the child number, which is unique for all radios. Therefore, it should not be interfering with operation.
- Maybe using the heap for the Roller* pointer is taking up too much RAM
  
- Am I setting return to transmitting false?
- In the radio_receive, maybe don't use an if statement for radio.available(...). Instead use a while loop?