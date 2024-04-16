# SPDX-License-Identifier: MPL-2.0
# Author: Sven Schneider
import os
import rdflib

def loader(directory):
    """
    Return a function that loads a file from the specified directory.
    """

    def load(file):
        with open(os.path.join(directory, file)) as f:
            return f.read()
        return ""

    return load

def for_type(*types):
    """
    Decorator that checks if a translator is applicable to a set of RDF types.
    The decorator must be applied to a class and injects a static method called
    "is_applicable" that accepts two arguments (an RDFLib graph and a node in
    that graph). If the types provided as arguments to the decorator are a
    subset of the node's rdf:type the function return true else false.
    """

    def decorator_for_type(cls):
        @staticmethod
        def is_applicable(g: rdflib.Graph, node: rdflib.URIRef) -> bool:
            return set(types) <= set(g[node : rdflib.RDF["type"]])

        cls.is_applicable = is_applicable
        return cls

    return decorator_for_type