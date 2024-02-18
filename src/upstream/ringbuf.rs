const BUFFER_SIZE: usize = 128; // Adjust the size as needed

pub(crate) struct RingBuffer<T> {
    buffer: [Option<T>; BUFFER_SIZE],
    head: usize,
    tail: usize,
    size: usize,
}

pub(crate) enum BufferError {
    Overflow,
    // Other error types can be added here if needed in the future
}

impl<T: core::marker::Copy> RingBuffer<T> {
    // Creates a new RingBuffer
    pub(crate) fn new() -> RingBuffer<T> {
        RingBuffer {
            buffer: [None; BUFFER_SIZE],
            head: 0,
            tail: 0,
            size: 0,
        }
    }

    // Adds an item to the buffer. Returns an error if the buffer is full.
    pub(crate) fn push(&mut self, item: T) -> Result<(), BufferError> {
        while self.size >= BUFFER_SIZE {
            self.discard();
        }
        self.buffer[self.tail] = Some(item);
        self.tail = (self.tail + 1) % BUFFER_SIZE;
        self.size += 1;
        Ok(())
    }

    // Peeks the next item in the buffer
    pub(crate) fn peek(&mut self) -> Option<&mut T> {
        if self.size > 0 {
            self.buffer[self.head].as_mut()
        } else {
            None
        }
    }

    // Discards the last item in the buffer
    pub(crate) fn discard(&mut self) {
        if self.size > 0 {
            self.buffer[self.head].take();
            self.head = (self.head + 1) % BUFFER_SIZE;
            self.size -= 1;
        }
    }
}
