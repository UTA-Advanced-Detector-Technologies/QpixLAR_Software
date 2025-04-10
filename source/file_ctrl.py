import numpy as np
import h5py

DATASET = "QPIX_EVENTS"

class datafile():
    def __init__(self):
        # output file data definitions are found here.
        self.cols = ['reset_mask', 'timestamp', 'metadata']

        # Define the structured data type
        self.dtype = np.dtype([
            (self.cols[0], np.uint16),  # 16-bit unsigned integer
            (self.cols[1], np.uint64),  # 64-bit unsigned integer
            (self.cols[2], np.int32)    # 32-bit signed integer
        ])
        
        self.f = None

    def open(self, file_name):
        self.f = h5py.File(file_name, "w")
        if DATASET not in self.f:
            self.dataset = self.f.create_dataset(DATASET, shape=(0,), maxshape=(None,), dtype=self.dtype)

    def close(self, file_name):
        del self.f

    def log_event(self, mask, timestamp, meta):
        """
        Append a new event entry to the dataset.
        """
        evt = np.array([(mask, timestamp, meta)], dtype=self.dtype)
        old_size = self.dataset.shape[0]
        self.dataset.resize((old_size+1,)) # Resize dataset
        self.dataset[old_size] = evt       # Store new event
