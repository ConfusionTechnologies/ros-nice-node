class Symbol:
    def __init__(self, name=""):
        self.name = f"<symbol {name}>"

    def __repr__(self):
        return self.name
