from tkinter import *
import os
from pathlib import Path


class StitchingWindow():
    def __init__(self) -> None:
        self.raw_pictures_path = Path(__file__).parent / 'pictures/raw'
        self.height = 400
        self.width = 400
        self.position = "400+400"
        self.root = Tk()
        self.root.withdraw()
        self.main_menu()
        self.root.mainloop()

    def new_window(self, title: str) -> Toplevel:
        window = Toplevel(self.root)
        window.title(title)
        window.geometry(f'{self.width}x{self.height}+{self.position}')
        return window

    def next_window(self, next_window_fnc) -> None:
        self.update_window_size_position(self.current_window)
        self.current_window.destroy()
        self.current_window.update()
        next_window_fnc()

    def main_menu(self):
        self.current_window = self.new_window('Stitching')
        frame = Frame(self.current_window, width=self.width, height=100, colormap="new")
        frame.pack(side=TOP)
        lbl = Label(frame, text="Press START to start the process to retrieve stitching parameters.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl.pack()
        btn = Button(self.current_window, text="START", fg='black',
                     command=lambda: self.next_window(self.take_pictures))
        btn.place(relx=0.5, rely=0.5, anchor=CENTER)

        btn_frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        btn_frame.pack(side=BOTTOM)
        self.place_quit_btn(btn_frame)

    def take_pictures(self) -> None:
        self.current_window = self.new_window('Take pictures')
        btn_frame = Frame(self.current_window, width=self.width,
                      height=0.5*self.height, colormap="new")        
        btn_frame.pack(side=BOTTOM)
        self.place_buttons(btn_frame,
                           self.main_menu, self.stitch)

        frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        frame.pack(side=TOP)
        
        lbl = Label(frame, text="Make sure the cameras needed are set correctly in config/config.yaml. Press START to start the script. When done, press the button 'q' to stop the loop, and the button 't' to save photos for the stitching process and to stop the loop.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl.pack()

        btn = Button(self.current_window, text="START", fg='black',
                     command=self.delete_existing_and_start_take_pictures)
        btn.place(relx=0.5, rely=0.5, anchor=CENTER)
    
    def delete_existing_and_start_take_pictures(self):
        os.system(f"rm -rf {self.raw_pictures_path}")
        os.system(f"python3 {os.path.dirname(os.path.realpath(__file__))}/take_picture.py")
        self.next_window(self.stitch)

    def stitch(self) -> None:
        self.current_window = self.new_window('Stitch')       
        btn_frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        btn_frame.pack(side=BOTTOM)
        self.place_buttons(btn_frame,
                           self.take_pictures, self.quit)

        frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        frame.pack(side=TOP)
        
        lbl = Label(frame, text="In this step, you will run the script to retrieve the stitching parameters for the images you collected in the previous step.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl.pack()
        lbl2 = Label(frame, text="To crop the result shown double click the desired lower right corner of the picture. To stop, press the button 'q'.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl2.pack()
        lbl3 = Label(frame, text="When satisfied with the result, press NEXT or QUIT to exit the GUI.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl3.pack()

        btn = Button(self.current_window, text="START", fg='black',
                     command=self.start_stitching)
        btn.place(relx=0.5, rely=0.5, anchor=CENTER)

    def start_stitching(self):
        os.system(f"python3 {os.path.dirname(os.path.realpath(__file__))}/stitch.py")

    def quit(self) -> None:
        self.root.destroy()

    def place_quit_btn(self, window: Toplevel) -> None:
        quit = Button(window, text="QUIT", fg='black', command=self.quit)
        quit.grid(row=0, column=3)

    def place_restart_btn(self, current_window: Toplevel) -> None:
        restart = Button(current_window, text="RESTART", fg='black',
                         command=lambda: self.next_window(self.main_menu))
        restart.grid(row=0, column=0)

    def place_back_btn(self, current_window: Toplevel, previous_window_fnc) -> None:
        back = Button(current_window, text="BACK", fg='black',
                      command=lambda: self.next_window(previous_window_fnc))
        back.grid(row=0, column=1)

    def place_next_btn(self, current_window: Toplevel, next_window_fnc) -> None:
        back = Button(current_window, text="NEXT", fg='black',
                      command=lambda: self.next_window(next_window_fnc))
        back.grid(row=0, column=2)

    def place_buttons(self, current_window: Toplevel, previous_window_fnc, next_window_fnc) -> None:
        self.place_quit_btn(current_window)
        self.place_back_btn(current_window, previous_window_fnc)
        self.place_next_btn(current_window, next_window_fnc)
        self.place_restart_btn(current_window)

    def update_window_size_position(self, window: Toplevel):
        self.position = f"{window.winfo_x()}+{window.winfo_y()-30}"
        self.height = window.winfo_height()
        self.width = window.winfo_width()



StitchingWindow()
