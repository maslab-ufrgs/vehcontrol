
class DecoratorClass(object):

    def __init__(self, decorated):
        self.__decorated = decorated

    def __getattr__(self, attr):
        return getattr(self.__decorated, attr)
