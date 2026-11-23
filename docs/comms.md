# Communication Design

### RX
- Use an RX worker thread to read incoming packets and push them onto a queue for processing

### TX
- Use a TX worker thread that contains a queue of outgoing packets
- Tasks that want to TX anything must submit their packets to the TX worker queue
