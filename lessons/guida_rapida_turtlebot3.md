# 🐢 Guida Rapida TurtleBot3 (Burger)

In ogni nuovo terminale aperto sul tuo computer, ricordati di impostare il modello del robot prima di lanciare qualsiasi comando:
```bash
export TURTLEBOT3_MODEL=burger
```

---

## 🅰️ Caso A: Simulazione (Gazebo)
Utilizza questa procedura per esercitarti in un ambiente virtuale 3D.

1.  **Avvia il Master (Il "Cervello")**
    Apri un terminale e digita:
    ```bash
    roscore
    ```
    *Mantieni questo terminale sempre aperto.*

2.  **Lancia l'ambiente virtuale**
    In un nuovo terminale, avvia il mondo di simulazione:
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

3.  **Inizializza il Robot (Bringup)**
    In un nuovo terminale, carica la descrizione e lo stato del robot:
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_remote.launch
    ```

4.  **Controllo Manuale (Teleop)**
    In un nuovo terminale, usa la tastiera per muovere il robot:
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
    *Usa i tasti **W, A, S, D, X** e tieni il terminale selezionato.*

5.  **Visualizzazione (Opzionale - RViz)**
    Per vedere cosa "pensa" il robot (dati LiDAR, ecc.):
    ```bash
    rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
    ```

---

## 🅱️ Caso B: Robot Reale (Il Bot Fisico)
Utilizza questa procedura quando lavori con il robot fisico. Assicurati che il tuo PC e il Bot siano sulla stessa rete Wi-Fi.

1.  **PC: Avvia il Master**
    Sul tuo computer:
    ```bash
    roscore
    ```

2.  **BOT: Inizializza l'Hardware**
    Collegati via SSH al robot e lancia i sensori e i motori:
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```

3.  **PC: Inizializzazione Remota**
    Sul tuo computer, per sincronizzare la comunicazione dei dati (necessario per il LiDAR):
    ```bash
    roslaunch turtlebot3_bringup turtlebot3_remote.launch
    ```

4.  **PC: Controllo Manuale**
    Sul tuo computer, avvia il comando da tastiera:
    ```bash
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```

5.  **PC: Visualizzazione (Opzionale - RViz)**
    Sul tuo computer, per vedere i punti rossi del LiDAR e la posizione del robot:
    ```bash
    rosrun rviz rviz
    ```
    *Nota: In RViz, imposta il "Fixed Frame" su `odom` per vedere correttamente i dati del sensore.*

---

### 💡 Consigli Utili
*   **Gazebo** serve per simulare la fisica del mondo.
*   **RViz** serve per visualizzare i sensori del robot.
*   Se il robot non si muove in Gazebo, controlla che la simulazione non sia in **Pausa** (tasto in basso al centro).
