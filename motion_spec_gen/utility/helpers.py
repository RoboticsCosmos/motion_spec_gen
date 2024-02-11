# SPDX-License-Identifier: MPL-2.0
# Author: Sven Schneider
import os


def loader(directory):
    """
    Return a function that loads a file from the specified directory.
    """

    def load(file):
        with open(os.path.join(directory, file)) as f:
            return f.read()
        return ""

    return load
