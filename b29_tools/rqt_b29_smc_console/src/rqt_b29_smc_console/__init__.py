__all__ = ['B29SmcConsolePlugin']


def __getattr__(name):
    if name == 'B29SmcConsolePlugin':
        from .console_plugin import B29SmcConsolePlugin

        return B29SmcConsolePlugin
    raise AttributeError(name)
