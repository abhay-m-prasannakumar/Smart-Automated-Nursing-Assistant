Smart Automated Nursing Assistant is seamless innovative project which reduces the worload of nurses and improves the effieciency of hospital management.
The detailed report with all requirements ,working and details is attached.Read the working before running the program
Harware components includes:
ESP32 Devkit
L298N Motor Driver
DC Motors and Wheels
LED
Push Button
3.7V Battery
Buzzer
Plastic storage rack
The code to be uploaded in ESP32 is provided in file minicode.ino ,assemble the components as shown in the circuit diagram and upload the codeto ESP32 then run the program.The navigation of trolley is controlled by A* algorithm.To operate the request giving functionality ,here we have used Blynk IOT software.Here is the breakdown of how to create the template in blynk console and the way to connect and control it. The image of the created blynk console is also provided.

 **Prerequisites:**
1. Create a Blynk account at [https://blynk.cloud](https://blynk.cloud)
2. Install Blynk Library if using ESP32/ESP8266 in Arduino IDE.
3. Have your device (ESP32/ESP8266) added to Blynk Console.

---
 **Step-by-Step Setup in Blynk.Console**
 **Step 1: Create a Template**
1. Go to **Developer Zone > Templates**.
2. Click **"New Template"**.
3. Set a name like `Nurseasst`, choose hardware (e.g., ESP32), and set connection type (Wi-Fi).
4. Save it.
 **Step 2: Create Datastreams**
1. Inside the template, go to the **"Datastreams"** tab.
2. Create three **Virtual Pin** datastreams:
   - `Room_101_priority` → V1 (Virtual Pin V1), Type: Integer, Min: 0, Max: 10
   - `Room_102_priority` → V2 (Virtual Pin V2), Type: Integer, Min: 0, Max: 10
   - `Room_103_priority` → V3 (Virtual Pin V3), Type: Integer, Min: 0, Max: 10
 **Step 3: Create a Device**
1. Go to **Devices > + New Device**.
2. Choose **"From Template"**, and select your template `Nurseasst`.
3. Name it and click **Create**.

**Step 4: Add Widgets in Web Dashboard**
1. Open your device (e.g., `Nurseasst`) and go to **Edit Dashboard**.
2. Click **+ Add Widget** and choose **Slider** or **Stepper** (Stepper is used in your screenshot).
3. Link each widget to the respective datastream:
   - Slider 1 → `Room_101_priority`
   - Slider 2 → `Room_102_priority`
   - Slider 3 → `Room_103_priority`
4. Adjust the Min/Max values if needed.
5. Click **Save and Exit Edit Mode**.

---

### 📲 (Optional) Mobile Dashboard
If you also want to control it from the Blynk app:
1. Open the Blynk app on your phone.
2. Go to your device and edit the mobile dashboard.
3. Add **Steppers/Sliders** and link them to V1, V2, and V3.

---
On setting the harware and software parts of the project, provide the power supply and when ESP32 turns on it gets connected to the provided wifi and the blynk device also gets online.Then you can perform  routine task on pressing push button and provide single or multiple requests via blynk and the trolley moves accordingly. You can adjust the time upto which the trolley should take requests and you can add and change room locations as per your choice.  
