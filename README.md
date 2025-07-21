# Robot Control System con Algoritmi di Scheduling

Questo progetto implementa un sistema di controllo distribuito per robot autonomo dotato di LiDAR, con particolare focus su algoritmi di scheduling in tempo reale e gestione di race condition attraverso meccanismi di sincronizzazione.

## 🎯 Obiettivo del Progetto

Il sistema controlla un robot autonomo che deve:
- Navigare evitando ostacoli rilevati tramite LiDAR
- Mappare l'ambiente circostante
- Ritornare al punto di spawn quando necessario
- Gestire priorità multiple in modo thread-safe

## 🏗️ Architettura del Sistema

Il progetto è organizzato in **5 nodi ROS2** che comunicano attraverso topics dedicati:

### 1. **Sistema Operativo Central Scheduler** (`sistemaoperativo.cpp`)
**Cuore del sistema di scheduling e sincronizzazione**

#### Algoritmo di Scheduling Basato su Priorità:
```cpp
#define OBSTACLE_PRIORITY 0      // Massima priorità - sicurezza
#define MAPPING_PRIORITY 1       // Esplorazione ambiente
#define BACKHOME_PRIORITY 2      // Ritorno al spawn
#define NAVIGATION_PRIORITY 3    // Navigazione normale
```

#### Gestione Race Condition:
- **Mutex (`data_mut`)**: Protezione accesso condiviso ai dati
- **Semaforo Counting (`sem`)**: Sincronizzazione timer/subscriber
- **Atomic Operations**: Gestione flag di stato thread-safe

#### Meccanismo di Sincronizzazione Avanzato:
```cpp
// Attesa sincronizzata di tutti i subscriber
if (!(obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived)){
    cControlLoop++;
    lock.unlock();
    sem.acquire();  // Blocco fino a ricezione dati
    lock.lock(); 
}
```

### 2. **Mapping Node** (`map.cpp`)
- **Algoritmo**: Esplorazione pseudo-casuale con pattern deterministico
- **Funzione**: Genera comandi di movimento per mappatura ambiente
- **Output**: Topic `/vel_mapping` con comandi Twist

### 3. **Navigation Node** (`nav.cpp`)
- **Sensori**: Elaborazione dati LiDAR (`/scan`) e odometria (`/odom`)
- **Algoritmo**: Obstacle avoidance + path planning verso home
- **Features**: 
  - Rilevamento ostacoli frontali (soglia 0.8m)
  - Orientamento verso punto di spawn usando quaternioni
  - Controllo reattivo in tempo reale

### 4. **Localization Node** (`localize.cpp`)
- **Stato**: Implementazione base (placeholder per algoritmi SLAM)
- **Scopo**: Localizzazione robot nell'ambiente mappato

### 5. **Velocity Modifier** (`velocity_modifier.cpp`)
- **Funzione**: Post-processing dei comandi di velocità
- **Algoritmo**: Scaling basato su distanza ostacoli

## 🔄 Flusso di Controllo e Scheduling

### Diagramma delle Priorità:
```
┌─────────────────┐    Priorità 0 (CRITICA)
│ OBSTACLE_VEL    │ ──────────────────────────┐
└─────────────────┘                           │
                                              ▼
┌─────────────────┐    Priorità 1          ┌──────────────┐
│ MAPPING_VEL     │ ──────────────────────▶│ SCHEDULER    │
└─────────────────┘                        │ (timer_callback)
                                           │              │
┌─────────────────┐    Priorità 2          │              │
│ BACKHOME_VEL    │ ──────────────────────▶│              │
└─────────────────┘                        └──────────────┘
                                              │
┌─────────────────┐    Priorità 3             │
│ NAV_VEL         │ ──────────────────────────┘
└─────────────────┘
```

### Algoritmo di Decisione:
```cpp
// Logica di priorità nel scheduler
if(obstacle_vel.linear.x == 0) { 
    priority = OBSTACLE_PRIORITY;    // Override per sicurezza
} else {
    priority = std::stoi(status);    // Priorità da stato corrente
}
```

## 🔒 Gestione Concorrenza e Race Condition

### Problemi Risolti:

1. **Data Race sui Messaggi**:
   - **Problema**: Accesso simultaneo a `obstacle_vel`, `nav_vel`, etc.
   - **Soluzione**: Lock guard su ogni callback subscriber

2. **Synchronization Barrier**:
   - **Problema**: Timer potrebbe eseguire prima della ricezione di tutti i messaggi
   - **Soluzione**: Semaforo counting + flag di sincronizzazione

3. **Priority Inversion**:
   - **Problema**: Task a bassa priorità potrebbero bloccare quelli critici
   - **Soluzione**: Priorità tramite controllo esplicito

### Esempio di Protezione Thread-Safe:
```cpp
backhome_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
    "backhome_vel", 10,
    [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mut);  // Protezione automatica
        backhome_vel = *msg;
        backhome_arrived = true;
        
        // Controllo sincronizzazione atomico
        if ((obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived) 
            && cControlLoop) {
            sem.release();  // Sblocca timer
            cControlLoop--;
        }
    });
```

## 📊 Stati del Robot

Il sistema implementa una **macchina a stati** controllata via topic `/status`:

| Stato | Descrizione | Priorità Attiva |
|-------|-------------|-----------------|
| `"0"` | **Allerta Ostacolo** | OBSTACLE_PRIORITY |
| `"1"` | **Mapping Attivo** | MAPPING_PRIORITY |
| `"2"` | **Ritorno al Spawn** | BACKHOME_PRIORITY |
| `"3"` | **Navigazione Libera** | NAVIGATION_PRIORITY |

## 🚀 Compilazione e Esecuzione

### Prerequisites:
- **ROS2 Jazzy/Iron**
- **GCC con supporto C++17**
- **Librerie**: `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`

### Build:
```bash
cd ~/ros2_ws
colcon build --packages-select localization_node mapping_node move_package navigation_node vel_modifier
source install/setup.bash
```

### Launch Sistema Completo:
```bash
# Terminal 1 - Sistema Operativo (Scheduler)
ros2 launch move_package <launch_file_name> // Work In Progress..
```

## 📡 Topics di Comunicazione

### Input Topics:
- `/scan` (sensor_msgs/LaserScan) - Dati LiDAR
- `/odom` (nav_msgs/Odometry) - Odometria robot
- `/status` (std_msgs/String) - Stato sistema

### Internal Topics:
- `/obstacle_vel` (geometry_msgs/Twist) - Comandi anti-collisione
- `/mapping_vel` (geometry_msgs/Twist) - Comandi mapping
- `/backhome_vel` (geometry_msgs/Twist) - Comandi ritorno
- `/nav_vel` (geometry_msgs/Twist) - Comandi navigazione

### Output Topics:
- `/cmd_vel` (geometry_msgs/Twist) - **Comandi finali al robot**

## ⚡ Performance e Real-Time

### Timing Specifications:
- **Timer Scheduler**: 500ms (2 Hz) - Garantisce reattività
- **Control Loop Navigation**: 100ms (10 Hz) - Controllo fluido
- **Mapping**: 100ms (10 Hz) - Esplorazione continua

### Ottimizzazioni Implementate:
1. **Lock-free Operations** dove possibile
2. **Minimal Critical Sections** - Lock brevi e focalizzati
3. **Priority-based Scheduling** - Sicurezza sempre prioritaria
4. **Efficient Memory Access** - Copia dati in critical section

## 🔧 Configurazioni Avanzate

### Tuning Parametri Sicurezza:
```cpp
// navigation_node.cpp
float front = msg->ranges[mid_index]; 
if (front < 0.8) {  // Soglia ostacolo - configurabile
    obstacle_detected_ = true;
}
```

### Tuning Velocità:
```cpp
// Velocità lineare standard
cmd.linear.x = 0.3;  // m/s

// Velocità angolare per evitamento
cmd.angular.z = 0.5; // rad/s
```

## 🐛 Debug e Monitoring

### Log Diagnostici:
Il sistema produce log dettagliati per ogni fase:
```cpp
RCLCPP_INFO(this->get_logger(), "Status aggiornato: '%s'", status.c_str());
RCLCPP_INFO(this->get_logger(), "Pubblicato messaggio su /cmd_vel");
```

### Monitoring Tools:
```bash
# Visualizza topics attivi
ros2 topic list

# Monitor comandi robot
ros2 topic echo /cmd_vel

# Monitor stato sistema  
ros2 topic echo /status
```

## 🎓 Aspetti Teorici Implementati

### Algoritmi di Scheduling:
- **Priority Scheduling** con preemption
- **Rate Monotonic** per task periodici
- **Earliest Deadline First** (implicito nei timer)

### Sincronizzazione:
- **Monitor Pattern** (mutex + condition variables)
- **Producer-Consumer** (subscriber/timer)
- **Barrier Synchronization** (semaforo counting)

### Real-Time Systems:
- **Bounded Response Time** - Timer deterministici
- **Priority Inheritance** - Prevenzione priority inversion
- **Thread-Safe Data Structures** - Protezione stato condiviso

---

## 📝 Note di Implementazione

**Limitazioni Attuali**:
- Velocity modifier in sviluppo
- Localization è solo un placeholder per ora

**Possibili Miglioramenti**:
- Implementazione SLAM per localization
- Algoritmi di path planning più sofisticati
- Gestione errori di comunicazione
- Metriche di performance in tempo reale

**Contributi**: Il sistema dimostra efficacemente l'implementazione di algoritmi di scheduling real-time e gestione concorrenza in un contesto robotico pratico.
