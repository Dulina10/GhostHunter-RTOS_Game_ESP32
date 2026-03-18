# 👻 Ghost Hunter – Real-Time Game using FreeRTOS

## 🌍 Project Overview
Ghost Hunter is a real-time embedded game developed using **ESP32** and **FreeRTOS**.

This project is focused on building a **time-sensitive interactive game** using real-time task handling, sensors, and embedded logic.

Main idea is simple:  
👉 Player must use **light correctly and at the correct time** to survive and banish the ghost.

---

## 🎮 Core Game Idea
The game is based on **light vs ghost interaction**.

- Player uses light as the main defense  
- Ghost appears and tries to attack  
- Player must react in real-time to survive  

This is not just a normal game — it depends on **timing, task execution, and system behavior**.

---

## 💡 LDR Logic (Important)
The LDR works in a **reverse way** in this system:

- LDR Covered → Light turns ON ✅  
- Light ON → Player can banish ghost  
- LDR Not Covered → Dark ❌  
- Dark → Ghost can appear and attack  

So player must control light manually using LDR.

---

## ⚡ Light Power System
Light is **not unlimited**.

- Holding LDR covered for long time → power decreases  
- When power goes low → light becomes weak / OFF  
- In that moment → ghost can attack  

So player must:
- Use light only when needed  
- Release LDR and wait → power recharge  

👉 This adds **strategy + timing**, not just simple ON/OFF gameplay.

---

## 🕹️ Player Controls
- **Joystick** → player movement / direction  
- **LDR** → control light  

Player must coordinate both to survive.

---

## 👾 Game Mechanics
- Ghost appears based on real-time game logic  
- Player must activate light at correct time  
- Correct timing → ghost banished  
- Wrong timing → ghost attacks  

Also:
- Light direction and player movement affect gameplay interaction  
- Real-time response is important for survival  

---

## 🧮 Scoring System
- ✔ Banish ghost → score increases  
- ❌ Fail → player gets attacked  
- ⏱ Survival time → increases difficulty  

Score depends on **player performance and reaction timing**.

---

## ⬆️ Level System
Level system is based on time.

- Every **20 seconds → level up**  
- Each level increases difficulty  
- Ghost becomes faster  
- Player must react quicker  

So in higher levels, gameplay feels faster and more intense.

---

## ⚙️ System Features
- Real-time execution using FreeRTOS  
- Multi-task system design  
- Sensor-based gameplay  
- Time-based events and difficulty  
- Serial monitor debugging  

---

## 🧠 FreeRTOS Implementation
The system is divided into **5 main tasks**:

- **LDR Task**  
  Reads LDR and controls light logic  

- **Display Task**  
  Updates game visuals (player, ghost, UI)  

- **Game Task**  
  Handles ghost behavior, scoring, level system  

- **Joystick Task**  
  Reads player input and movement  

- **Verbose Task**  
  Outputs debug information to serial monitor  

All tasks run concurrently, making the system behave as a **real-time application**.

---

## 🧩 Hardware Used
- ESP32  
- LDR Sensor  
- Joystick Module  
- LEDs / Game Display  

---

## 📂 Project Structure
- `code/` → source code  
- `images/` → screenshots  
- `videos/` → gameplay demo  
- `presentation/` → slides  

---

## ⚠️ Limitations
- Some parts can be smoother  
- Graphics are basic  
- Sensor depends on environment lighting  
- Gameplay tuning can be improved  

---

## 🚀 Future Improvements
- smoother gameplay and transitions  
- improved graphics / display  
- sound effects  
- better optimization of timing  
- extended game features  

---

## 👤 Author
**Dulina Nadith**  
BEng (Hons) Electrical and Electronic Engineering  
University of Hertfordshire (UK)  
SLTM Nebula Institute of Technology
