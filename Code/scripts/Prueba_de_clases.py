class upper_class(object):
    def __init__(self):
        a = 10
        self.b = 15

        lower_class = self.lower_class(self)

        self.b = 50

        lower_class.funcion_de_prueba_lower()

        print self.b

        print lower_class.c


    def funcion_de_prueba_upper(self):
        print "hola"
        self.b = 20

    class lower_class(object):
        def __init__(self,inherit):
            # print a
            print inherit.b
            self.inherit = inherit
            inherit.funcion_de_prueba_upper()

        def funcion_de_prueba_lower(self):
            print "adios"
            self.c = 30
            print self.inherit.b
            self.inherit.funcion_de_prueba_upper()


f = upper_class().funcion_de_prueba_upper

f()