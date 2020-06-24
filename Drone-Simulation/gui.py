import tkinter as tk

class gui:
    # simulation parameters
    t = 0
    dt = 0.01

    # gui general parameters
    TITLE = "Drone Simulation"
    GUI_WIDTH = 1100
    GUI_HEIGHT = 700
    GUI_TOP_LEFT_CORNER_X = 50
    GUI_TOP_LEFT_CORNER_Y = 50
    BUTTON_WIDTH = 10


    # frame size parameters
    DISPLAY_CANVAS_WIDTH = 800
    DISPLAY_CANVAS_HEIGHT = 500

    # drone perspective parameters
    SCALE_FACTOR = 100
    WORLD_WIDTH = DISPLAY_CANVAS_WIDTH/SCALE_FACTOR
    WORLD_HEIGHT = DISPLAY_CANVAS_HEIGHT/SCALE_FACTOR

    LOW_FRAMES_HEIGHT = GUI_HEIGHT - DISPLAY_CANVAS_HEIGHT

    SIDE_FRAMES_WIDTH = GUI_WIDTH - DISPLAY_CANVAS_WIDTH
    SIDE_FRAMES_HEIGHT = DISPLAY_CANVAS_HEIGHT

    # gui styles
    DISPLAY_BG_COLOUR = "white"
    BUTTON_BG_COLOUR = "white"
    FRAME_BG_COLOUR = "gray"
    SLIDER_BG_COLOUR = "gray"
    GAINS_BG_COLOUR = "gray"
    BORDER_COLOUR = "black"
    BORDER_WIDTH = 1
    TITLE_TEXT_SIZE = 18
    NORMAL_TEXT_SIZE = 10
    FONT = "TkDefaultFont"
    TEXT_COLOUR = "black"

    # GUI Status Variables
    START_PRESSED = False
    EXIT_PRESSED = False
    LOG_DATA_PRESSED = False
    SAVE_LOGGED_DATA = False
    NEW_INIT_CON = False
    HOVER_DYNAMICS = True


    # Set initial parameters for opening
    ICs = [4,0,2,0,0,69]
    initial_gains = [3,2,9,3,0.1,0.4]

    def getGuiStatus(self):
        if self.EXIT_PRESSED:
            return False
        return True


    def initialiseFrames(self):
        self.uiFrame = tk.Frame(self.gui, bg=self.FRAME_BG_COLOUR, height=self.LOW_FRAMES_HEIGHT,
                width=self.GUI_WIDTH, bd=self.BORDER_WIDTH,highlightbackground=self.BORDER_COLOUR,
                highlightcolor=self.BORDER_COLOUR, highlightthickness=self.BORDER_WIDTH)


        self.uiFrame.grid(row=1,column=0,columnspan=2)

        self.displayCanvas = tk.Canvas(self.gui, bg=self.DISPLAY_BG_COLOUR, height=self.DISPLAY_CANVAS_HEIGHT,
                width=self.DISPLAY_CANVAS_WIDTH,highlightbackground=self.BORDER_COLOUR,
                highlightcolor=self.BORDER_COLOUR, highlightthickness=self.BORDER_WIDTH)
        self.displayCanvas.grid(row=0,column=0)

        self.flightDataFrame = tk.Frame(self.gui, bg=self.FRAME_BG_COLOUR,
                height=self.SIDE_FRAMES_HEIGHT, width=self.SIDE_FRAMES_WIDTH,
                highlightbackground=self.BORDER_COLOUR, highlightcolor=self.BORDER_COLOUR,
                highlightthickness=self.BORDER_WIDTH)
        self.flightDataFrame.grid(row=0,column=1)

    def startButtonCallback(self,event):
        if self.START_PRESSED:
            self.startButton['text'] = "Start"
        else:
            self.startButton['text'] = "Pause"
        self.START_PRESSED = not self.START_PRESSED
        print(self.START_PRESSED)

    def restartButtonCallback(self,event):
        # self.t = 0
        print("Restart simulation")

    def exitButtonCallback(self,event):
        print("Closing GUI")
        self.EXIT_PRESSED = True

    def saveButtonCallback(self):
        print("Data Saved")

    def disturbancesButtonCallback(self):
        print("Disturbances added")

    def updateGainsButtonCallback(self):
        print("Update Gains")
        

    def updateIniticialConditions(self):
        print("Update ICs")
        self.ICs = [
            self.x_IC.get(),
            self.x_dot_IC.get(),
            self.y_IC.get(),
            self.y_dot_IC.get(),
            self.theta_IC.get(),
            self.theta_dot_IC.get()
        ]

    def initialiseControlButtons(self):
        BUTTON_COL_L = 0
        BUTTON_COL_R = 1

        TITLE_ROW = 0
        ROW_1 = 1
        ROW_2 = 2
        ROW_3 = 3
        ROW_4 = 4

        BUTTON_PAD_X = 10
        BUTTON_PAD_Y = 1

        title = tk.Label(self.uiFrame, bg=self.FRAME_BG_COLOUR, text="Control Buttons")
        title.grid(row=TITLE_ROW,columnspan=len([BUTTON_COL_L,BUTTON_COL_R]))

        self.startButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Start", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.startButtonCallback)
        self.startButton.grid(column=BUTTON_COL_L,padx=BUTTON_PAD_X,row=ROW_1,pady=BUTTON_PAD_Y)

        self.restartButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Restart", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.restartButtonCallback)
        self.restartButton.grid(column=BUTTON_COL_L,padx=BUTTON_PAD_X,row=ROW_2,
                pady=BUTTON_PAD_Y)

        self.exitButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Exit", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.exitButtonCallback)
        self.exitButton.grid(column=BUTTON_COL_L,padx=BUTTON_PAD_X,row=ROW_3,
                pady=BUTTON_PAD_Y)



        self.plotButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Plot Data", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.startButtonCallback)
        self.plotButton.grid(column=BUTTON_COL_R,padx=BUTTON_PAD_X,row=ROW_1,
                pady=BUTTON_PAD_Y)

        self.saveButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Save Data", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.saveButtonCallback)
        self.saveButton.grid(column=BUTTON_COL_R,padx=BUTTON_PAD_X,row=ROW_2,
                pady=BUTTON_PAD_Y)

        self.disturbancesButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Disturbances", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.disturbancesButtonCallback)
        self.disturbancesButton.grid(column=BUTTON_COL_R,padx=BUTTON_PAD_X,row=ROW_3,
                pady=BUTTON_PAD_Y)



        self.gui.bind('<space>', self.startButtonCallback)
        self.gui.bind('<Escape>', self.exitButtonCallback)
        self.gui.bind('<BackSpace>', self.restartButtonCallback)

    def initialiseReferences(self):
        SLIDER_COL = 6
        SLIDER_LENGTH = 150
        SLIDER_WIDTH = 15
        SLIDER_PAD_X = 7
        SLIDER_PAD_Y = 1
        X_ROW = 0
        Y_ROW = 3
        ROW_SPAN = 3

        self.x_ref_slider = tk.Scale(self.uiFrame, from_=0, to=self.WORLD_WIDTH,
            width=SLIDER_WIDTH, length=SLIDER_LENGTH,bg=self.SLIDER_BG_COLOUR,
            tickinterval=0.5,orient="horizontal",label="X Reference")
        self.x_ref_slider.set(4)
        self.x_ref_slider.grid(column=SLIDER_COL,row=X_ROW,rowspan=ROW_SPAN,
            padx=SLIDER_PAD_X,pady=SLIDER_PAD_Y)

        self.y_ref_slider = tk.Scale(self.uiFrame, from_=0, to=self.WORLD_HEIGHT,
            width=SLIDER_WIDTH, length=SLIDER_LENGTH,bg=self.SLIDER_BG_COLOUR,
            tickinterval=0.5,orient="horizontal",label="Y reference")
        self.y_ref_slider.set(2)
        self.y_ref_slider.grid(column=SLIDER_COL,row=Y_ROW,rowspan=ROW_SPAN,
            padx=SLIDER_PAD_X,pady=SLIDER_PAD_Y)

    def initialiseFlightData(self):
        Z = [4,0,2,0,0,0]
        F = [2,69]

        X_ROW = 1
        Y_ROW = 2
        THETA_ROW = 3
        THRUST_ROW = 5

        POS_TEXT_COL = 0
        POS_COL = 1
        VEL_TEXT_COL = 3
        VEL_COL = 4
        ACC_TEXT_COL = 6
        ACC_COL = 7

        # labelFrame = tk.Frame(self.flightDataFrame,bg=self.FRAME_BG_COLOUR, width=self.SIDE_FRAMES_WIDTH)
        flightDataTitle = tk.Label(self.flightDataFrame,text="Flight Data",
                bg=self.FRAME_BG_COLOUR,font=(self.FONT,self.TITLE_TEXT_SIZE)).grid(row=0,columnspan=8)



        x_label = tk.Label(self.flightDataFrame,text="x =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=X_ROW,column=POS_TEXT_COL)
        x = tk.StringVar()
        x.set("%.2fm" % Z[0])
        x_pos = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=x,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        x_pos.grid(row=X_ROW,column=POS_COL)


        x_dot_label = tk.Label(self.flightDataFrame,text="x_d =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=X_ROW,column=VEL_TEXT_COL)
        x_dot = tk.StringVar()
        x_dot.set("%.2fm" % Z[1])
        x_vel = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=x_dot,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        x_vel.grid(row=X_ROW,column=VEL_COL)


        x_dd_label = tk.Label(self.flightDataFrame,text="x_dd =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=X_ROW,column=ACC_TEXT_COL)
        x_dd = tk.StringVar()
        x_dd.set("?m/s^2")
        x_acc = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=x_dd,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        x_acc.grid(row=X_ROW,column=ACC_COL)



        y_label = tk.Label(self.flightDataFrame,text="y =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=Y_ROW,column=POS_TEXT_COL)
        y = tk.StringVar()
        y.set("%.2fm" % Z[2])
        y_pos = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=y,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        y_pos.grid(row=Y_ROW,column=POS_COL)

        y_dot_label = tk.Label(self.flightDataFrame,text="y_d =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=Y_ROW,column=VEL_TEXT_COL)
        y_dot = tk.StringVar()
        y_dot.set("%.2fm" % Z[3])
        y_vel = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=y_dot,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        y_vel.grid(row=Y_ROW,column=VEL_COL)

        y_dd_label = tk.Label(self.flightDataFrame,text="y_dd=",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=Y_ROW,column=ACC_TEXT_COL)
        y_dd = tk.StringVar()
        y_dd.set("?m/s^2")
        y_acc = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=y_dd,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        y_acc.grid(row=Y_ROW,column=ACC_COL)



        theta_label = tk.Label(self.flightDataFrame,text="Theta =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=THETA_ROW,column=POS_TEXT_COL)
        theta = tk.StringVar()
        theta.set("%.2fm" % Z[4])
        theta_pos = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=theta,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        theta_pos.grid(row=THETA_ROW,column=POS_COL)

        theta_dot_label = tk.Label(self.flightDataFrame,text="Theta_d =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=THETA_ROW,column=VEL_TEXT_COL)
        theta_dot = tk.StringVar()
        theta_dot.set("%.2fm" % Z[5])
        theta_vel = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=theta_dot,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        theta_vel.grid(row=THETA_ROW,column=VEL_COL)

        theta_dd_label = tk.Label(self.flightDataFrame,text="Theta_dd =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=THETA_ROW,column=ACC_TEXT_COL)
        theta_dd = tk.StringVar()
        theta_dd.set("?m/s^2")
        theta_acc = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=theta_dd,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        theta_acc.grid(row=THETA_ROW,column=ACC_COL)


        Fr_label = tk.Label(self.flightDataFrame,text="F_r =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=THRUST_ROW,column=POS_TEXT_COL)
        F_r = tk.StringVar()
        F_r.set("%.2fN" % F[0])
        Thrust_right = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=F_r,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        Thrust_right.grid(row=THRUST_ROW,column=POS_COL)


        Fl_label = tk.Label(self.flightDataFrame,text="F_l =",bg=self.FRAME_BG_COLOUR,
                font=(self.FONT,self.NORMAL_TEXT_SIZE)).grid(row=THRUST_ROW,column=VEL_TEXT_COL)
        F_l = tk.StringVar()
        F_l.set("%.2fN" % F[1])
        Thrust_left = tk.Label(self.flightDataFrame,bg=self.FRAME_BG_COLOUR,textvariable=F_l,
                font=(self.FONT,self.NORMAL_TEXT_SIZE))
        Thrust_left.grid(row=THRUST_ROW,column=VEL_COL)


    def setInitialConditions(self):


        POS_TEXT_COL = 2
        POS_COL = 3
        VEL_TEXT_COL = 4
        VEL_COL = 5

        LABEL_WIDTH = 10
        ENTRY_WIDTH = 5
        COL_SPAN = 4

        ENTRY_PAD_X = 3

        TITLE_ROW = 0
        X_ROW = 1
        Y_ROW = 2
        THETA_ROW = 3
        BUTTON_ROW = 4

        title = tk.Label(self.uiFrame, bg=self.FRAME_BG_COLOUR, text="Set ICs")
        title.grid(row=TITLE_ROW,column=POS_TEXT_COL,columnspan=COL_SPAN)

        self.updateIniticialConditionsButton = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Set ICs", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.updateIniticialConditions)
        self.updateIniticialConditionsButton.grid(column=POS_TEXT_COL,row=BUTTON_ROW,columnspan=COL_SPAN)


        x_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="X(m):",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        x_label.grid(row=X_ROW,column=POS_TEXT_COL)

        self.x_IC = tk.DoubleVar()
        self.x_IC.set(self.ICs[0])
        x_entry = tk.Entry(self.uiFrame,textvariable=self.x_IC,width=ENTRY_WIDTH)
        x_entry.grid(row=X_ROW,column=POS_COL,padx=ENTRY_PAD_X)


        y_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Y(m):",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        y_label.grid(row=Y_ROW,column=POS_TEXT_COL)

        self.y_IC = tk.DoubleVar()
        self.y_IC.set(self.ICs[2])
        y_entry = tk.Entry(self.uiFrame,textvariable=self.y_IC,width=ENTRY_WIDTH)
        y_entry.grid(row=Y_ROW,column=POS_COL,padx=ENTRY_PAD_X)


        theta_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Theta (Rad):",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        theta_label.grid(row=THETA_ROW,column=POS_TEXT_COL)

        self.theta_IC = tk.DoubleVar()
        self.theta_IC.set(self.ICs[4])
        theta_entry = tk.Entry(self.uiFrame,textvariable=self.theta_IC,width=ENTRY_WIDTH)
        theta_entry.grid(row=THETA_ROW,column=POS_COL,padx=ENTRY_PAD_X)





        x_dot_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="X_d (m/s)",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        x_dot_label.grid(row=X_ROW,column=VEL_TEXT_COL)

        self.x_dot_IC = tk.DoubleVar()
        self.x_dot_IC.set(self.ICs[1])
        x_dot_entry = tk.Entry(self.uiFrame,textvariable=self.x_dot_IC,width=ENTRY_WIDTH)
        x_dot_entry.grid(row=X_ROW,column=VEL_COL,padx=ENTRY_PAD_X)


        y_dot_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Y_d (m/s)",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        y_dot_label.grid(row=Y_ROW,column=VEL_TEXT_COL)

        self.y_dot_IC = tk.DoubleVar()
        self.y_dot_IC.set(self.ICs[3])
        y_dot_entry = tk.Entry(self.uiFrame,textvariable=self.y_dot_IC,width=ENTRY_WIDTH)
        y_dot_entry.grid(row=Y_ROW,column=VEL_COL,padx=ENTRY_PAD_X)


        theta_dot_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Theta_d (rad/s)",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        theta_dot_label.grid(row=THETA_ROW,column=VEL_TEXT_COL)

        self.theta_dot_IC = tk.DoubleVar()
        self.theta_dot_IC.set(self.ICs[5])
        theta_dot_entry = tk.Entry(self.uiFrame,textvariable=self.theta_dot_IC,width=ENTRY_WIDTH)
        theta_dot_entry.grid(row=THETA_ROW,column=VEL_COL,padx=ENTRY_PAD_X)


    def initialiseGains(self):
        K = [2,0,4,0,0,0]

        KP_TEXT_COL = 7
        KP_COL = 8
        KD_TEXT_COL = 9
        KD_COL = 10

        LABEL_WIDTH = 7
        ENTRY_WIDTH = 5
        COL_SPAN = 4

        ENTRY_PAD_X = 3

        TITLE_ROW = 0
        X_ROW = 1
        Y_ROW = 2
        THETA_ROW = 3
        BUTTON_ROW = 4

        title = tk.Label(self.uiFrame, bg=self.FRAME_BG_COLOUR, text="Update Gains")
        title.grid(row=TITLE_ROW,column=KP_TEXT_COL,columnspan=COL_SPAN)

        self.updateICs = tk.Button(self.uiFrame,  bg=self.BUTTON_BG_COLOUR,
                text="Update Gains", fg=self.TEXT_COLOUR, width=self.BUTTON_WIDTH,
                command=self.updateGainsButtonCallback)
        self.updateICs.grid(column=KP_TEXT_COL,row=BUTTON_ROW,columnspan=COL_SPAN)


        Kp_x_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="X: Kp",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        Kp_x_label.grid(row=X_ROW,column=KP_TEXT_COL)

        self.Kp_x = tk.StringVar()
        self.Kp_x.set(K[0])
        Kp_x_entry = tk.Entry(self.uiFrame,textvariable=self.Kp_x,width=ENTRY_WIDTH)
        Kp_x_entry.grid(row=X_ROW,column=KP_COL,padx=ENTRY_PAD_X)

        Kp_y_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Y: Kp",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        Kp_y_label.grid(row=Y_ROW,column=KP_TEXT_COL)

        self.Kp_y = tk.StringVar()
        self.Kp_y.set(K[2])
        Kp_y_entry = tk.Entry(self.uiFrame,textvariable=self.Kp_y,width=ENTRY_WIDTH)
        Kp_y_entry.grid(row=Y_ROW,column=KP_COL,padx=ENTRY_PAD_X)


        Kp_theta_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Theta: Kp",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        Kp_theta_label.grid(row=THETA_ROW,column=KP_TEXT_COL)

        self.Kp_theta = tk.StringVar()
        self.Kp_theta.set(K[4])
        Kp_theta_entry = tk.Entry(self.uiFrame,textvariable=self.Kp_theta,width=ENTRY_WIDTH)
        Kp_theta_entry.grid(row=THETA_ROW,column=KP_COL,padx=ENTRY_PAD_X)





        Kd_x_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="X: Kd",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        Kd_x_label.grid(row=X_ROW,column=KD_TEXT_COL)

        self.Kd_x = tk.StringVar()
        self.Kd_x.set(K[1])
        Kd_x_entry = tk.Entry(self.uiFrame,textvariable=self.Kd_x,width=ENTRY_WIDTH)
        Kd_x_entry.grid(row=X_ROW,column=KD_COL,padx=ENTRY_PAD_X)


        Kd_y_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Y: Kd",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        Kd_y_label.grid(row=Y_ROW,column=KD_TEXT_COL)

        self.Kd_y = tk.StringVar()
        self.Kd_y.set(K[3])
        Kd_y_entry = tk.Entry(self.uiFrame,textvariable=self.Kd_y,width=ENTRY_WIDTH)
        Kd_y_entry.grid(row=Y_ROW,column=KD_COL,padx=ENTRY_PAD_X)


        Kd_theta_label = tk.Label(self.uiFrame,bg=self.GAINS_BG_COLOUR,text="Theta: Kd",
                    font=(self.FONT,self.NORMAL_TEXT_SIZE),width=LABEL_WIDTH)
        Kd_theta_label.grid(row=THETA_ROW,column=KD_TEXT_COL)

        self.Kd_theta = tk.StringVar()
        self.Kd_theta.set(K[5])
        Kd_theta_entry = tk.Entry(self.uiFrame,textvariable=self.Kd_theta,width=ENTRY_WIDTH)
        Kd_theta_entry.grid(row=THETA_ROW,column=KD_COL,padx=ENTRY_PAD_X)

    def initialiseWindow(self):
        self.gui.title(self.TITLE)
        self.gui.geometry("{}x{}+{}+{}".format(self.GUI_WIDTH,self.GUI_HEIGHT,self.GUI_TOP_LEFT_CORNER_X,self.GUI_TOP_LEFT_CORNER_Y))

    def __init__(self):
        self.gui = tk.Tk()
        self.initialiseWindow()
        self.initialiseFrames()
        # self.initialiseFlightData()
        self.initialiseControlButtons()
        self.initialiseReferences()
        self.initialiseGains()
        self.setInitialConditions()
        self.updateFrameSizes()
        self.draw_compass()

    def draw_compass(self):
        x_origin = 10
        y_origin = self.DISPLAY_CANVAS_HEIGHT-10
        length = 50

        x_arrow = self.displayCanvas.create_line(x_origin, y_origin, x_origin+length, y_origin,
            arrow=tk.LAST)
        y_arrow = self.displayCanvas.create_line(x_origin, y_origin, x_origin, y_origin-length,
            arrow=tk.LAST)

        x_origin += 80
        y_origin -= 10
        self.displayCanvas.create_text(x_origin,y_origin,text="(Xmax = {}m, Ymax = {}m)".format(self.WORLD_WIDTH,self.WORLD_HEIGHT))

    def updateFrameSizes(self):
        self.gui.update()
        self.gui.update_idletasks()
        PADDING_HEIGHT = (self.LOW_FRAMES_HEIGHT - self.uiFrame.winfo_height())/2
        PADDING_WIDTH = (self.GUI_WIDTH - self.uiFrame.winfo_width())/2
        self.uiFrame.configure(padx=PADDING_WIDTH,pady=PADDING_HEIGHT)
