import logging
from logging.handlers import RotatingFileHandler
import time

class log:
    def __init__(self, filepath):
        self.filepath = filepath
        self.setup_logging(filepath)

    def setup_logging(self, filepath: str):
        self.logger = logging.getLogger(filepath)
        #self.logger.setLevel(logging.INFO)
        if not self.logger.hasHandlers():
            # create a rotating file handler
            handler = RotatingFileHandler(filepath, maxBytes=100000, backupCount=5)
            handler.setLevel(logging.INFO)
            # create a formatter and set it for the handler
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            # Add the handler to the logger
            self.logger.addHandler(handler)
            # ensure basicConfig does not override our custom settings
            logging.basicConfig(level=logging.INFO, handlers=[handler])