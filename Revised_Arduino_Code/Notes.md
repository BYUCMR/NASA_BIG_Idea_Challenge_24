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

## 1/31/2025
I have noticed that the arduinos potentially act differently when connected to the computer, even though their main power supply remains the battery. I feel like this might have something to do with grounding then? I figure that the computer would be able to provide a consistent grounding, and this might help performance.

Node 4 is busted. It saps power from my computer, and will not allow it to connect to the serial monitor at all.

I was testing the "right" side of the robot ie down node 3, but then node 2 was asking node 1 for information again. When I plugged into node 1, node 2 kept on asking. Thus, I believe it was node 2 that was mistaken about its data. Even though node 1 was trying to communicate back, node 2 was not belieiving it. Therefore, check the error code to see if it gets stuck in an infinite loop or not.

# 2/6/2025
I swapped out Node 4 and it's working like a charm.

Node 2 does not like it when it is powered with battery and then plugged into computer. Could be an issue with how the arduino is drawing power. It's green light flashes which indicates that the radio is thinking it is receiving something. Node 2 has had some issues in the past (see previous comments), so this might be another symptom of the same problem. In fact, as I wrote this sentence (ie not sending any commands), it started to flash green at an absurd rate and seemingly "receive" something. It also was requesting reinformation from node 1, as confirmed by the fact that once node 2 was restarted, node 1 would stop flashing its retransmit. Therefore, I am going to replace node 2 as well.

Add to README
- Note about different arduino bootloader