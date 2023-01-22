from tkinter import *
import os
from PIL import Image, ImageTk
from pathlib import Path


class CameraCalibraton():
    def __init__(self) -> None:
        self.raw_pictures_path = Path(__file__).parent / 'pictures/raw'
        self.fixed_pictures_path = Path(__file__).parent / 'pictures/fixed'
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
        self.current_window = self.new_window('Camera calibration')
        frame = Frame(self.current_window, width=100, height=100, colormap="new")
        frame.pack(side=TOP)
        lbl = Label(frame, text="Press START to start the calibration process",
                    fg='black', font=("Helvetica", 12))
        lbl.pack()
        btn = Button(self.current_window, text="START", fg='black',
                     command=lambda: self.next_window(self.take_pictures))
        btn.place(relx=0.5, rely=0.5, anchor=CENTER)

        btn_frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        btn_frame.pack(side=BOTTOM)
        self.place_quit_btn(btn_frame)

    def take_pictures(self) -> None:
        cams = (os.scandir("/dev")) 
        cams = ([i.path for i in cams if "video" in i.path])
        self.current_window = self.new_window('Step 1')
        btn_frame = Frame(self.current_window, width=self.width,
                      height=0.5*self.height, colormap="new")        
        btn_frame.pack(side=BOTTOM)
        self.place_buttons(btn_frame,
                           self.main_menu, self.show_pictures)

        frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        frame.pack(side=TOP)
        
        lbl = Label(frame, text="Place the checkered board along the edges of the camera to take pictures needed for calibration. Choose a camera to calibrate from the dropdown menu and press START to start the process. Press the button q to stop taking photos.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl.pack()

        self.cam = StringVar()
        drop = OptionMenu(frame, self.cam,  *cams )
        drop.pack()
        lbl2 = Label(frame, text="Enter the name of the output config file:")
        lbl2.pack()
        self.output = StringVar()
        self.output.set("camera.json")
        entry = Entry(frame, textvariable=self.output)
        entry.pack()
        btn = Button(self.current_window, text="START", fg='black',
                     command=self.delete_existing_and_start_take_pictures)
        btn.place(relx=0.5, rely=0.5, anchor=CENTER)
    
    def delete_existing_and_start_take_pictures(self):
        os.system(f"rm -rf {self.raw_pictures_path}")
        os.system(f"python3 {os.path.dirname(os.path.realpath(__file__))}/take_pictures.py {self.cam.get()}")
        self.next_window(self.show_pictures)

    def show_pictures(self):
        self.current_window = self.new_window('Calibration pictures')
        frame = self.display_images(self.raw_pictures_path)
        self.place_buttons(frame, self.take_pictures, self.calibrate)


    def calibrate(self) -> None:
        self.current_window = self.new_window('Calibrate')       
        btn_frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        btn_frame.pack(side=BOTTOM)
        self.place_buttons(btn_frame,
                           self.show_pictures, self.undistort)

        frame = Frame(self.current_window, width=self.width,
                      height=0.3*self.height, colormap="new")        
        frame.pack(side=TOP)
        
        lbl = Label(frame, text="In this step, you will run the calibration script to undistort the images you collected in the previous step.",
                    fg='black', font=("Helvetica", 12), wraplength=self.width)
        lbl.pack()

        btn = Button(self.current_window, text="START", fg='black',
                     command=self.start_calibration)
        btn.place(relx=0.5, rely=0.5, anchor=CENTER)

    def start_calibration(self):
        self.update_window_size_position(self.current_window)
        self.current_window.destroy()
        self.current_window.update()
        os.system(f"python3 {os.path.dirname(os.path.realpath(__file__))}/calibrate.py {self.output.get()}")
        os.system(f"python3 {os.path.dirname(os.path.realpath(__file__))}/undistort.py")
        self.undistort()

    def undistort(self) -> None:
        self.current_window = self.new_window('Undistort')
        frame = self.display_images(self.fixed_pictures_path)
        self.place_buttons(frame, self.calibrate, self.quit)

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

    def display_images(self, path):
        width, height = [int(i) for i in self.get_screen_size(self.current_window)]
        self.current_window.geometry(f"{width}x{height}")
        self.update_window_size_position(self.current_window)

        frame = Frame(self.current_window, width= width, height=100)
        frame.grid(row=1, column=0, sticky='n')

        canvas = Canvas(self.current_window, width = 0.955*width, height = (1-100/height)*height)
        canvas.grid(row=0, column=0, sticky= "news")

        vsb = Scrollbar(self.current_window, orient="vertical", command=canvas.yview)
        vsb.grid(row=0, column=1, sticky="ns")

        frame_image = Frame(canvas)

        if os.path.exists(path):

            pictures = [file.path for file in os.scandir(path) if "png" in file.path]      
            
            row = 0
            column = 0
            for pic in pictures:
                image = Image.open(pic)
                size_x, size_y = tuple([int(width/3/image.size[0]*x) for x in image.size])
                photo = (ImageTk.PhotoImage(image.resize((size_x, size_y))))
                label = Label(frame_image, image=photo)
                label.image = photo
                label.grid(row=row, column=column, sticky='news')
                if column == 2:
                    row += 1
                    column = -1
                column+=1
    
        canvas.create_window((0,0), window=frame_image, anchor="nw")
        canvas.update_idletasks()
        canvas.config(scrollregion=canvas.bbox("all"),yscrollcommand= vsb.set)

        return frame
        
    def get_screen_size(self, window: Toplevel) -> tuple:
        return (window.winfo_screenwidth(), window.winfo_screenheight())


CameraCalibraton()
