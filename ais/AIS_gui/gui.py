from tkinter import *
from tkinter import filedialog
from tkinter import ttk
from datetime import datetime
import tkinter as tk
import gmplot
import webbrowser
import split as sp

global entry_folder
global entry_lat
global entry_lon
global entry_RAN
global entry_FROM
global entry_TO
global entry_file
global entry_split_FROM
global entry_split_TO
global entry_file_TO

class Gui(tk.Tk):
    def browseFolders(self):
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
     
    def browseFile(self):
        file_name = filedialog.askopenfilename()
        self.entry_file.delete(0, END)
        self.entry_file.insert(0, file_name) 
    
    def browseFile_TO(self):
        file_name = filedialog.askopenfilename()
        self.entry_file_TO.delete(0, END)
        self.entry_file_TO.insert(0, file_name)
   
    def splitByTime(self):
        path_r = self.entry_file.get()
        path_w = self.entry_file_TO.get()
        start_time = self.entry_split_FROM.get()
        end_time = self.entry_split_TO.get()
        print
        sp.splitByTime(start_time, end_time, path_r, path_w)
    
    def splitByRange(self):
        pass


    def __init__(self):
        
        super().__init__()
        self.title("Select Folder")
        self.geometry("800x400")
        self.resizable(0,0)
        self.config(background = "black")
        tabControl = ttk.Notebook(self)
        tab1    = ttk.Frame(tabControl)
        tab2    = ttk.Frame(tabControl)
        tabControl.add(tab1,    text= "Read")
        tabControl.add(tab2,    text= "Split File")
        tabControl.pack(expand = 1, fill = "both")
        ##TAB Read #############################################################
        
        label_POS = Label(tab1,
                        text    = "Position",
                        ).grid(column = 1, row = 1, ipadx=5, pady=5, sticky=E)
            #----------------------------------------------------------
        label_lat = Label(tab1,
                        text    = "Lat:",
                        ).grid(column = 2, 
                                row = 1, 
                                ipadx =5 , 
                                pady = 5, 
                                sticky = W)
            #----------------------------------------------------------
        self.entry_lat = Entry(tab1, width = 10)
        self.entry_lat.insert(0, "24.04185")
        self.entry_lat.grid(column = 3, row = 1, ipadx = 5, pady = 5, sticky = W)
            #----------------------------------------------------------
        label_lon = Label(tab1,
                        text    = "Lon:",
                        ).grid(column = 4, 
                                row = 1, 
                                ipadx =5 , 
                                pady = 5, 
                                sticky = W)
            #----------------------------------------------------------
        self.entry_lon = Entry(tab1, width = 10)
        self.entry_lon.insert(0, "120.33571")
        self.entry_lon.grid(column = 5, row = 1, ipadx = 5, pady = 5, sticky = W)

        #################################################################

        label_RAN = Label(tab1,
                        text    = "Range(km)",
                        ).grid(column = 1, 
                                row = 2, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E)
            #--------------------------------------------------------------
        self.entry_RAN = Entry(tab1, width = 20)
        self.entry_RAN.insert(0, "15")
        self.entry_RAN.grid(column = 2, row = 2, columnspan = 3)

        ##################################################################

        label_FROM = Label(tab1,
                        text    = "From(UTC)",
                        ).grid(column = 1, 
                                row = 3, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E)
            #------------------------------------------------------------
        self.entry_FROM = Entry(tab1, width = 20)
        self.entry_FROM.insert(0, "2021-09-17 23:45:00")
        self.entry_FROM.grid(column = 2, row = 3, columnspan = 3)

            #------------------------------------------------------------
        label_FROM_eg   = Label(tab1, 
                            text    = "format: YYYY-MM-DD HH:MM:SS",
                            ).grid(column = 5, 
                                    row = 3, 
                                    ipadx = 5, 
                                    pady = 5, 
                                    sticky=W)
        ####################################################################

        label_TO = Label(tab1,
                        text    = "To(UTC)",
                        ).grid(column = 1, 
                                row = 4, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E)
            #---------------------------------------------------------------
        self.entry_TO = Entry(tab1, width = 20)
        self.entry_TO.insert(0, "2021-09-17 23:59:59")
        self.entry_TO.grid(column = 2, row = 4, columnspan = 3)

        #######################################################################

        label_FOLDER = Label(tab1,
                        text    = "File Folder",
                        ).grid(column = 1, 
                                row = 5, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E)

            #------------------------------------------------------------------

        self.entry_folder = Entry(tab1, width = 20)
        self.entry_folder.insert(0, "/folder/path")
        self.entry_folder.grid(column = 2, row = 5, columnspan = 3)

            #-------------------------------------------------------------------

        button_exp = Button(tab1,
                            text    = "Browse Folders",
                            bg      = "white",
                            fg      = "black",
                            command = self.browseFolders
                            ).grid(column = 5, 
                                    row = 5,
                                    ipadx = 5,
                                    pady    = 5,
                                    sticky=W)

        #########################################################################

        button_exit = Button(tab1,
                            text    = "Exit",
                            bg      = "red",
                            fg      = "white",
                            command = exit).grid(column = 3, row = 6)

        button_apply = Button(tab1,
                            text    = "Apply",
                            bg      = 'blue',
                            fg      = 'white',
                            command = self.getInput).grid(column = 1, row = 6)

        
        ##TAB Split#######################################################################
        label_split_FROM = Label(tab2,
                        text    = "From(UTC)",
                        ).grid(column = 1, 
                                row = 3, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E
                                )
            #------------------------------------------------------------
        self.entry_split_FROM = Entry(tab2, width = 20)
        self.entry_split_FROM.grid(column = 2,row = 3, columnspan = 3)
        label_FROM_eg   = Label(tab2, 
                            text    = "format: YYYY-MM-DD HH:MM:SS",
                            ).grid(column = 5, 
                                    row = 3, 
                                    ipadx = 5, 
                                    pady = 5, 
                                    sticky=W)
        self.entry_split_FROM.insert(0,"2021-09-01 01:25:00")
        ####################################################################

        label_splt_TO = Label(tab2,
                        text    = "To(UTC)",
                        ).grid(column = 1, 
                                row = 4, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E
                                )
            #---------------------------------------------------------------
        self.entry_split_TO = Entry(tab2, width = 20)
        self.entry_split_TO.grid(column = 2, row = 4, columnspan = 3)
        self.entry_split_TO.insert(0, "2021-09-01 01:26:00")

        ########################################################################
        label_FILE_R = Label(tab2,
                        text    = "From csv File",
                        ).grid(column = 1, 
                                row = 5, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E)

            #------------------------------------------------------------------

        self.entry_file = Entry(tab2, width = 20)
        self.entry_file.insert(0, "/folder/path/file.csv")
        self.entry_file.grid(column = 2, row = 5, columnspan = 3)

            #-------------------------------------------------------------------
        ########################################################################
        label_FILE_W = Label(tab2,
                        text    = "New file path and name",
                        ).grid(column = 1, 
                                row = 6, 
                                ipadx=5, 
                                pady=5, 
                                sticky=E)

            #------------------------------------------------------------------

        self.entry_file_TO = Entry(tab2, width = 20)
        self.entry_file_TO.insert(0, "/folder/path/new.csv")
        self.entry_file_TO.grid(column = 2, row = 6, columnspan = 3)

            #-------------------------------------------------------------------



    



        button_file = Button(tab2,
                            text    = "Select File",
                            bg      = "white",
                            fg      = "black",
                            command = self.browseFile
                            ).grid(column = 5, 
                                    row = 5,
                                    ipadx = 5,
                                    pady  = 5,
                                    sticky=W) 

        button_file = Button(tab2,
                            text    = "Select File",
                            bg      = "white",
                            fg      = "black",
                            command = self.browseFile_TO
                            ).grid(column = 5, 
                                    row = 6,
                                    ipadx = 5,
                                    pady  = 5,
                                    sticky=W)

        button_file = Button(tab2,
                            text    = "Split File",
                            bg      = "blue",
                            fg      = "white",
                            command = self.splitByTime
                            ).grid(column = 1, 
                                    row = 7,
                                    )

        button_exit = Button(tab2,
                            text    = "Exit",
                            bg      = "red",
                            fg      = "white",
                            command = exit
                            ).grid(column = 3, 
                                    row = 7)


if __name__ == "__main__":
    gui = Gui()
    gui.mainloop()
