import Tkinter as tk
from runTens import TensegrityMotorController, Tensegrity

WIDTH = 640
HEIGHT = 480

class TensGUI(object):
    """Class for the tensegrity GUI Tkinter app."""

    def __init__(self, master):
        self.f = tk.Frame(master)
        self.f.grid(sticky=tk.N+tk.E+tk.S+tk.W)
        self.tensController = Tensegrity()
        self.__create_widgets()
        self.f._root().mainloop()

    def __create_widgets(self):
        """Create the initial widgets on the screen."""
        ## TITLE
        self.titleLabel = tk.Label(self.f, text="Tensegrity GUI",
                                   font=("Helvetica", 20))
        self.titleLabel.grid(row=0, column=0, columnspan=2,
                             sticky=tk.N+tk.E+tk.S+tk.W)

        ## ADD STRUT BOX
        self.addStrutFrame = tk.Frame(self.f, relief=tk.SUNKEN)
        self.addrLabel = tk.Label(self.addStrutFrame, text="BT Addr: ")
        self.addrTextbox = tk.Entry(self.addStrutFrame)
        self.addStrutButton = tk.Button(self.addStrutFrame, text="Add")

        self.addStrutFrame.grid(row=1, column=0, sticky=tk.N+tk.E+tk.S+tk.W)
        self.addrLabel.grid(row=0, column=0, sticky=tk.N+tk.E+tk.S+tk.W)
        self.addrTextbox.grid(row=0, column=1, sticky=tk.N+tk.E+tk.S+tk.W)
        self.addStrutButton.grid(row=1, column=0, columnspan=2, sticky=tk.N+tk.E+tk.S+tk.W)

        ## STRUT LIST
        self.strutListFrame = tk.Frame(self.f, relief=tk.RAISED)
        



if __name__ == '__main__':
    root = tk.Tk()
    tensGUI = TensGUI(root)
