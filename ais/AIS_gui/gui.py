from tkinter import *
from tkinter import filedialog
import tkinter as tk
import gmplot
import webbrowser


global entry_folder
global entry_lat
global entry_lon
global entry_RAN
global entry_FROM
global entry_TO


class Gui(tk.Tk):
    def browseFiles(self):
        folder_name = filedialog.askdirectory()
        self.entry_folder.delete(0, END)
        self.entry_folder.insert(0, folder_name) 

    def drawMap(self,lat,lon,ran,folder):
        gmapOne = gmplot.GoogleMapPlotter(lat,lon,12)
        gmapOne.marker(float(lat),float(lon),'blue')
        gmapOne.circle(lat,lon,ran)

        url = folder+"/target.html"
        if url != "/folder/path/target.html":
            gmapOne.draw(url)
            webbrowser.open(url)
        else:
            print("please fill in the path")

    def getInput(self):
        _lat = self.entry_lat.get()
        _lon = self.entry_lon.get()
        _ran = int(self.entry_RAN.get())*1000
        _from= self.entry_FROM.get()
        _to  = self.entry_TO.get()
        _dir = self.entry_folder.get()
        
        drawit = True
        if drawit:
            self.drawMap(_lat,_lon,_ran,_dir) 
            print(_lat, _lon, _ran, _from, _to, _dir)
    
    def __init__(self):
        
        super().__init__()
        self.title("Select Folder")
        self.geometry("800x800")
        self.config(background = "black")

        ###############################################################

        label_POS = Label(self,
                        text    = "Position",
                        bg      = "black",
                        fg      = "white")
        label_POS.grid(column = 1, row = 1, ipadx=5, pady=5, sticky=E)
            #----------------------------------------------------------
        label_lat = Label(self,
                        text    = "Lat:",
                        bg      = "black",
                        fg      = "white")
        label_lat.grid(column = 2, row = 1, ipadx =5 , pady = 5, sticky = W)
            #----------------------------------------------------------
        self.entry_lat = Entry(self, width = 10)
        self.entry_lat.insert(0, "24.04185")
        self.entry_lat.grid(column = 3, row = 1, ipadx = 5, pady = 5, sticky = W)
            #----------------------------------------------------------
        label_lon = Label(self,
                        text    = "Lon:",
                        bg      = "black",
                        fg      = "white")
        label_lon.grid(column = 4, row = 1, ipadx =5 , pady = 5, sticky = W)
            #----------------------------------------------------------
        self.entry_lon = Entry(self, width = 10)
        self.entry_lon.insert(0, "120.33571")
        self.entry_lon.grid(column = 5, row = 1, ipadx = 5, pady = 5, sticky = W)

        #################################################################

        label_RAN = Label(self,
                        text    = "Range(km)",
                        bg      = "black",
                        fg      = "white")
        label_RAN.grid(column = 1, row = 2, ipadx=5, pady=5, sticky=E)
            #--------------------------------------------------------------
        self.entry_RAN = Entry(self, width = 20)
        self.entry_RAN.insert(0, "15")
        self.entry_RAN.grid(column = 2, row = 2, columnspan = 3)

        ##################################################################

        label_FROM = Label(self,
                        text    = "From(UTC)",
                        bg      = "black",
                        fg      = "white")
        label_FROM.grid(column = 1, row = 3, ipadx=5, pady=5, sticky=E)
            #------------------------------------------------------------
        self.entry_FROM = Entry(self, width = 20)
        self.entry_FROM.insert(0, "YYYY/MM/DD/hh/mm/ss")
        self.entry_FROM.grid(column = 2, row = 3, columnspan = 3)
        ####################################################################

        label_TO = Label(self,
                        text    = "To(UTC)",
                        bg      = "black",
                        fg      = "white")
        label_TO.grid(column = 1, row = 4, ipadx=5, pady=5, sticky=E)
            #---------------------------------------------------------------
        self.entry_TO = Entry(self, width = 20)
        self.entry_TO.insert(0, "YYYY/MM/DD/hh/mm/ss")
        self.entry_TO.grid(column = 2, row = 4, columnspan = 3)

        #######################################################################

        label_FOLDER = Label(self,
                        text    = "File Folder",
                        bg      = "black",
                        fg      = "white")
        label_FOLDER.grid(column = 1, row = 5, ipadx=5, pady=5, sticky=E)

            #------------------------------------------------------------------

        self.entry_folder = Entry(self, width = 20)
        self.entry_folder.insert(0, "/folder/path")
        self.entry_folder.grid(column = 2, row = 5, columnspan = 3)

            #-------------------------------------------------------------------

        button_exp = Button(self,
                            text    = "Browse Folders",
                            bg      = "white",
                            fg      = "black",
                            command = self.browseFiles)
        button_exp.grid(column = 5, row = 5)

        #########################################################################

        button_exit = Button(self,
                            text    = "Exit",
                            bg      = "red",
                            fg      = "white",
                            command = exit)
        button_exit.grid(column = 3, row = 6)

        button_apply = Button(self,
                            text    = "Apply",
                            bg      = 'blue',
                            fg      = 'white',
                            command = self.getInput)
        button_apply.grid(column = 1, row = 6)

if __name__ == "__main__":
    gui = Gui()
    gui.mainloop()
