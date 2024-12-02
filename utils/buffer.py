class Buffer:
    def __init__(self, size):
        self.size = size
        self.buffer = []

    def append(self, item):
        if len(self.buffer) >= self.size:
            self.buffer.pop(0)
        self.buffer.append(item)

    def average(self):
        return sum(self.buffer) / len(self.buffer)
    
    def __sum__(self, other):
        return self.buffer.append(other)
        

    def __len__(self):
        return len(self.buffer)