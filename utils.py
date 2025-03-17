import os

def get_files_with_extension(directory, extension):
    return [f for f in os.listdir(directory) if f.endswith(extension)]
