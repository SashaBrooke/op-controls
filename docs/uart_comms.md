# UART Communication Design

### TX
- Use a TX worker thread that contains a queue of outgoing packet payloads (union of payload structs)
- Tasks that want to TX anything must submit their payload to the TX worker queue
- TX worker will dequeue payloads and package them into packets (single place setting the header)
