#!/usr/bin/env python

import rospy

# We use a try-except block for Python 2/3 compatibility with Tkinter
try:
    # Python 3
    import tkinter as tk
except ImportError:
    # Python 2
    import Tkinter as tk

import sys

# --- Configuration ---
# You can still use 'from_' and 'to' for documentation or future validation
SLIDER_CONFIG = {
    "k_d":  {"param": "/hrisim/weights/k_d",  "from_": 0.0, "to": 50.0},
    "k_bc": {"param": "/hrisim/weights/k_bc", "from_": 0.0, "to": 200.0},
    "k_pd": {"param": "/hrisim/weights/k_pd", "from_": 0.0, "to": 200.0}
}
# ---------------------

class WeightTunerGUI:
    def __init__(self, root):
        self.root = root
        root.title("Heuristic Weight Tuner")

        self.entries = {}
        
        # Create a frame for the widgets
        frame = tk.Frame(root)
        frame.pack(padx=15, pady=15)
        
        # Configure grid
        frame.grid_columnconfigure(1, weight=1) # Allow entry column to expand

        rospy.loginfo("[WeightTuner] GUI started. Getting initial param values...")

        row_index = 0
        # Create a label and entry box for each config entry
        for name, config in SLIDER_CONFIG.items():
            param_name = config["param"]
            
            # 1. Get the current value from the param server to set the initial text
            try:
                initial_val = rospy.get_param(param_name)
            except (KeyError, rospy.ROSException):
                rospy.logwarn("[WeightTuner] Param '%s' not set. Defaulting to 0.0" % (param_name))
                initial_val = 0.0


            # 2. Create the label
            label = tk.Label(frame, text=name)
            label.grid(row=row_index, column=0, padx=5, pady=5, sticky="w")

            # 3. Create the Entry (textbox)
            entry = tk.Entry(frame, width=15)
            entry.insert(0, str(initial_val)) # Set initial text
            entry.grid(row=row_index, column=1, columnspan=2, padx=5, pady=5, sticky="ew") # Use columnspan=2
            self.entries[name] = entry # Save entry widget to access it later

            # 4. Individual "Set" Buttons have been removed as requested.

            row_index += 1
        
        set_all_button = tk.Button(
            frame,
            text="Set",
            command=self.update_all_params,
            font=('Helvetica', 10, 'bold')
        )
        set_all_button.grid(row=row_index, column=0, columnspan=3, pady=10, sticky="ew")

        rospy.loginfo("[WeightTuner] Initialized. Ready to tune.")

    def update_param(self, name, param_name):
        """
        This callback is triggered by "update_all_params".
        It reads from the corresponding Entry widget, validates,
        and updates the ROS Parameter Server.
        """
        try:
            # 1. Get value from the correct Entry widget
            entry_widget = self.entries[name]
            value_str = entry_widget.get()
            
            # 2. Validate and cast to float
            value_float = float(value_str)
            
            # 3. Set the parameter
            rospy.set_param(param_name, value_float)
            rospy.loginfo("[WeightTuner] Set %s to %f" % (param_name, value_float))
            
        except ValueError:
            rospy.logerr("[WeightTuner] Invalid input '%s' for %s. Please enter a valid number." % (value_str, name))
        except rospy.ROSException as e:
            rospy.logerr("[WeightTuner] Failed to set param: %s" % e)
        except KeyError:
            rospy.logerr("[WeightTuner] Internal error: No Entry widget found for name %s" % name)

    def update_all_params(self):
        """Calls update_param for all configured parameters."""
        rospy.loginfo("[WeightTuner] Setting all parameters...")
        for name, config in SLIDER_CONFIG.items():
            self.update_param(name, config["param"])

def main():
    # We must initialize a ROS node for this GUI
    try:
        rospy.init_node('weight_tuner_gui', anonymous=True)
    except rospy.ROSException as e:
        print("Error initializing ROS node: %s" % e)
        print("Please make sure roscore is running.")
        sys.exit(1)

    # Create the main Tkinter window
    root = tk.Tk()
    app = WeightTunerGUI(root)
    
    # Start the Tkinter event loop
    # This loop runs the GUI
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.loginfo("[WeightTuner] Shutting down...")

if __name__ == '__main__':
    main()