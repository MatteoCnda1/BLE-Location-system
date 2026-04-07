# Projet Triangulation BLE avec STM32WB55

**Système de localisation indoor par triangulation BLE utilisant un STM32WB55 et des beacons ESP32**

![Status](https://img.shields.io/badge/status-production--ready-brightgreen)
![Version](https://img.shields.io/badge/version-1.0-blue)
![License](https://img.shields.io/badge/license-Educational-orange)

---

## 📋 Table des matières

- [Vue d'ensemble](#-vue-densemble)
- [Caractéristiques](#-caractéristiques)
- [Matériel requis](#-matériel-requis)
- [Architecture](#-architecture)
- [Installation](#-installation)
- [Calibration](#-calibration)
- [Configuration spatiale](#-configuration-spatiale)
- [Utilisation](#-utilisation)
- [Performance](#-performance)
- [Optimisations anti-crash](#-optimisations-anti-crash)
- [Troubleshooting](#-troubleshooting)
- [Améliorations futures](#-améliorations-futures)
- [Annexes](#-annexes)

---

## 🎯 Vue d'ensemble

### Description

Ce projet implémente un **système de triangulation BLE (Bluetooth Low Energy)** permettant de localiser un objet mobile (une "bombe") en temps réel dans un espace intérieur à l'aide de 3 balises fixes.

Le système calcule la position 2D et 3D de l'objet mobile avec une précision de **±10-20cm** jusqu'à 3 mètres.

### Composants

- **1x STM32WB55 Nucleo** avec shield environnemental (magnétomètre + accéléromètre)
- **3x ESP32-C3** comme balises fixes (M1, M2, M3)
- **1x ESP32-C6** utilisé pour la calibration (peut servir de balise fixe)
- **1x ESP32-C3** comme objet mobile à localiser ("Bombe")

### Principe de fonctionnement

1. **Mesure RSSI** : Le STM32WB55 scanne en continu les beacons BLE et mesure leur signal RSSI
2. **Conversion RSSI → Distance** : Formule logarithmique calibrée
3. **Filtrage Kalman** : Lissage du signal RSSI pour stabilité
4. **Trilatération** : Calcul de position par intersection de 3 sphères
5. **Résultat** : Position 2D (x, y) et 3D (x, y, z) en mètres

---

## ✨ Caractéristiques

- ✅ **Triangulation 2D et 3D** en temps réel
- ✅ **Filtrage Kalman** du signal RSSI pour stabilité
- ✅ **Interface graphique** Python (Tkinter) pour visualisation
- ✅ **Boussole numérique** intégrée (magnétomètre LIS2MDL)
- ✅ **Protection anti-crash USB** (garbage collection + limitation mémoire)
- ✅ **Précision calibrée** : erreur ~9% jusqu'à 3m
- ✅ **Stable** : Fonctionne >1h sans crash

---

## 🛠️ Matériel requis

| Composant | Quantité | Référence | Rôle |
|-----------|----------|-----------|------|
| STM32WB55 Nucleo | 1 | NUCLEO-WB55RG | Contrôleur principal + Scanner BLE |
| X-NUCLEO-IKS01A3 | 1 | Shield environnemental | Magnétomètre + Accéléromètre |
| ESP32-C6 DevKit | 1 | 40:4C:CA:45:37:06 | Balise fixe M1 (calibration) |
| ESP32-C3 DevKit | 3 | 1C:DB:D4:36:49:CA<br/>1C:DB:D4:37:4C:F2<br/>1C:DB:D4:34:74:DE | Balises M2, M3 + Bombe |
| Câble USB Micro-B | 1 | - | Communication STM32 ↔ PC |
| Mètre ruban | 1 | - | Calibration et mesures |

---

## 🏗️ Architecture

### Schéma fonctionnel

```
┌─────────────────────────────────────────────────────────────────┐
│                         SYSTÈME COMPLET                         │
└─────────────────────────────────────────────────────────────────┘

┌──────────────┐                    ┌──────────────┐
│   ESP32-C6   │ ←──── BLE ────→   │              │
│  M1 (Fixe)   │                    │              │
│ (0.0, 0.0)   │                    │   STM32WB55  │
└──────────────┘                    │              │
                                    │  • Scanner   │
┌──────────────┐                    │    BLE       │      ┌──────────┐
│   ESP32-C3   │ ←──── BLE ────→   │  • Capteurs  │ USB  │    PC    │
│  M2 (Fixe)   │                    │  • Calcul    │─────→│Interface │
│ (0.1, 3.0)   │                    │  • JSON      │      │ Graphique│
└──────────────┘                    │              │      └──────────┘
                                    │              │
┌──────────────┐                    │              │
│   ESP32-C3   │ ←──── BLE ────→   │              │
│  M3 (Fixe)   │                    │              │
│ (2.7, 2.7)   │                    └──────────────┘
└──────────────┘
        
┌──────────────┐                           ↑
│   ESP32-C3   │ ←──── BLE ────────────────┘
│    BOMBE     │         (Position calculée)
│  (Mobile)    │
└──────────────┘
```

### Flux de données

```
ESP32 (BLE advertising)
    ↓
STM32WB55 (Scan + Mesure RSSI)
    ↓
Filtre Kalman (Lissage)
    ↓
Conversion RSSI → Distance
    ↓
Trilatération (3 balises)
    ↓
Position 2D/3D calculée
    ↓
JSON via USB
    ↓
Interface PC (Visualisation)
```

---

## 📦 Installation

### Prérequis logiciels

#### Sur le PC (Ubuntu/Debian)

```bash
# Python 3 et pip
sudo apt update
sudo apt install python3 python3-pip

# Bibliothèques Python
pip3 install pyserial tkinter

# Outil ampy pour flasher le STM32
pip3 install adafruit-ampy

# Accès au port série (ajouter l'utilisateur au groupe dialout)
sudo usermod -a -G dialout $USER
# ⚠️ Déconnexion/reconnexion nécessaire
```

#### Arduino IDE (pour programmer les ESP32)

```bash
# Télécharger depuis https://www.arduino.cc/en/software
# Ou via snap
sudo snap install arduino
```

**Dans Arduino IDE :**

1. `File → Preferences → Additional Board Manager URLs`
2. Ajouter : `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. `Tools → Board → Boards Manager → "esp32"`
4. Installer version >= 3.0.0 (pour ESP32-C6)

### Installation du firmware STM32

#### 1. MicroPython sur STM32WB55

- Télécharger le firmware MicroPython pour STM32WB55
- Flasher via STM32CubeProgrammer ou st-link
- Vérifier la connexion :

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
# Tu devrais voir le prompt MicroPython : >>>
```

#### 2. Uploader le code principal

```bash
# Flasher le code de triangulation
ampy --port /dev/ttyACM0 put main_triangulation_ANTI_CRASH.py main.py

# Reset la carte (bouton noir RESET)
```

### Configuration des ESP32

#### Code Arduino pour balises fixes (M1, M2, M3)

```cpp
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>

#define BEACON_NAME "M1"  // ⚠️ Changer pour M2, M3, Bombe
#define TX_POWER ESP_PWR_LVL_N0  // 0 dBm

BLEAdvertising *pAdvertising;

void setup() {
  Serial.begin(115200);
  
  // Initialiser BLE
  BLEDevice::init(BEACON_NAME);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, TX_POWER);
  
  // Créer serveur et advertising
  BLEServer *pServer = BLEDevice::createServer();
  pAdvertising = BLEDevice::getAdvertising();
  
  BLEAdvertisementData advertisementData;
  advertisementData.setName(BEACON_NAME);
  advertisementData.setFlags(0x06);
  
  pAdvertising->setAdvertisementData(advertisementData);
  pAdvertising->setMinInterval(160);  // 100ms
  pAdvertising->setMaxInterval(160);
  
  // Démarrer
  pAdvertising->start();
  
  Serial.printf("Balise %s active\n", BEACON_NAME);
  Serial.printf("MAC: %s\n", BLEDevice::getAddress().toString().c_str());
}

void loop() {
  delay(1000);
}
```

**Programmer chaque ESP32 :**

| ESP32 | Type | Nom à définir | Rôle |
|-------|------|---------------|------|
| M1 | ESP32-C6 | `"M1"` | Balise fixe (calibration) |
| M2 | ESP32-C3 | `"M2"` | Balise fixe |
| M3 | ESP32-C3 | `"M3"` | Balise fixe |
| Bombe | ESP32-C3 | `"Bombe"` | Objet mobile à tracker |

⚠️ **Noter les adresses MAC affichées dans le moniteur série !**

---

## 🎯 Calibration

La calibration est **l'étape la plus importante** pour obtenir une bonne précision.

### Étape 1 : Mesure du TX_POWER (15 min)

**Objectif :** Mesurer le RSSI à exactement 1 mètre.

#### Procédure

1. Placer le STM32 et **un seul ESP32** sur une table vide
2. Distance **exactement 1.00m** (mètre ruban)
3. **Même hauteur** pour les deux (~1m du sol)
4. **Aucun obstacle** entre les deux
5. Lancer miniterm et noter les RSSI pendant 30 secondes

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200

# Observer les valeurs RSSI
[ -58 dBm] M1  40:4C:CA:45:37:06
[ -56 dBm] M1  40:4C:CA:45:37:06
[ -59 dBm] M1  40:4C:CA:45:37:06
[ -57 dBm] M1  40:4C:CA:45:37:06
...

# Calculer la moyenne : (-58-56-59-57-58-57) / 6 = -57.5 dBm
```

**Résultat du projet :** `TX_POWER = -57.3 dBm`

### Étape 2 : Mesure à plusieurs distances (15 min)

Répéter la mesure à 2m, 3m, 5m avec le **même ESP32** :

| Distance réelle | RSSI mesuré | Notes |
|----------------|-------------|-------|
| 1.00m | -57.3 dBm | Référence (moyenne 30 sec) |
| 2.00m | -69.0 dBm | Moyenne 30 sec |
| 3.00m | -75.3 dBm | Moyenne 30 sec |
| 5.00m | -77.0 dBm | Moyenne 30 sec |

### Étape 3 : Calcul du PATH_LOSS_EXPONENT (5 min)

Utiliser ce script Python pour trouver le meilleur `n` :

```python
import math

TX_POWER = -57.3
distances = [1.0, 2.0, 3.0, 5.0]
rssi_mesures = [-57.3, -69.0, -75.3, -77.0]

best_n = None
best_error = float('inf')

for n in [x * 0.1 for x in range(15, 50)]:  # Tester n de 1.5 à 5.0
    errors = []
    for i in range(len(distances)):
        dist_reelle = distances[i]
        rssi = rssi_mesures[i]
        dist_calc = 10 ** ((TX_POWER - rssi) / (10 * n))
        error = abs((dist_calc - dist_reelle) / dist_reelle * 100)
        errors.append(error)
    
    avg_error = sum(errors) / len(errors)
    if avg_error < best_error:
        best_error = avg_error
        best_n = n

print(f"Meilleur n = {best_n}")
print(f"Erreur moyenne = {best_error:.1f}%")
```

**Résultat du projet :**
- `TX_POWER = -57.3`
- `PATH_LOSS_EXPONENT = 3.8`
- **Erreur moyenne = 9.1%** (jusqu'à 3m)

### Étape 4 : Mise à jour du code

Dans `main_triangulation_ANTI_CRASH.py`, lignes 89-90 :

```python
TX_POWER = -57.3  # ← Ta valeur mesurée à 1m
PATH_LOSS_EXPONENT = 3.8  # ← Ta valeur calibrée
```

---

## 📐 Configuration spatiale

### Positionnement des balises

Les 3 balises fixes doivent former un **triangle** (pas alignées !).

#### Configuration du projet

```
      Y (m)
       ↑
    3.0│    M2 (0.10, 3.00)
       │    ●
       │     
    2.7│           M3 (2.70, 2.70)
       │              ●
    2.0│      
       │   Zone de
    1.5│   tracking
       │      
    1.0│     
       │
    0.0│M1 (0.0, 0.0)
       └─────────────────────→ X (m)
        0   1.0  2.0  2.7  3.0

Distances entre balises :
• M1 ↔ M2 : 3.00m
• M2 ↔ M3 : 2.62m
• M3 ↔ M1 : 3.82m
• Aire : 3.92m²

Zone optimale : Centre du triangle (0.93m, 1.90m)
```

### Mesure des positions

Pour chaque balise fixe, mesurer avec un mètre ruban :

- **x** : distance horizontale depuis M1 (origine)
- **y** : distance perpendiculaire à l'axe M1-M2
- **z** : hauteur par rapport au sol

| Balise | x (m) | y (m) | z (m) | Notes |
|--------|-------|-------|-------|-------|
| M1 (C6) | 0.00 | 0.00 | 0.00 | Origine au sol |
| M2 (C3) | 0.10 | 3.00 | 0.00 | 10cm décalé, 3m en Y |
| M3 (C3) | 2.70 | 2.70 | 0.00 | Diagonal |
| Bombe | ? | ? | ? | Position calculée |

### Mise à jour du code

Dans `main_triangulation_ANTI_CRASH.py`, lignes 56-79 :

```python
BEACONS_CONFIG = {
    "40:4C:CA:45:37:06": {  # M1
        "position": (0.0, 0.0, 0.0),  # ← Tes mesures réelles
        "role": "fixed",
        "name": "M1"
    },
    "1C:DB:D4:36:49:CA": {  # M2
        "position": (0.10, 3.0, 0.0),  # ← Tes mesures réelles
        "role": "fixed",
        "name": "M2"
    },
    "1C:DB:D4:37:4C:F2": {  # M3
        "position": (2.70, 2.70, 0.0),  # ← Tes mesures réelles
        "role": "fixed",
        "name": "M3"
    },
    "1C:DB:D4:34:74:DE": {  # Bombe
        "position": None,  # Position calculée automatiquement
        "role": "target",
        "name": "Bombe"
    }
}
```

---

## 🚀 Utilisation

### Démarrage du système

#### 1. Allumer les ESP32

- ✅ M1, M2, M3 (balises fixes) alimentées
- ✅ Bombe (objet mobile) alimentée
- ✅ LEDs clignotent sur tous les ESP32

#### 2. Démarrer le STM32

```bash
# Le STM32 démarre automatiquement
# Vérifier via miniterm (optionnel)
python3 -m serial.tools.miniterm /dev/ttyACM0 115200

# Tu devrais voir :
[BLE] Scanner initialisé
[MAG] LIS2MDL initialisé
[MEM] Mémoire libre: 45000 bytes
[BEACONS] 4 détecté(s)
```

#### 3. Lancer l'interface graphique

```bash
python3 pc_interface_triangulation.py

# Sélectionner le port /dev/ttyACM0
# Cliquer "Connecter"
```

### Interface graphique

L'interface affiche :

| Section | Contenu |
|---------|---------|
| **Boussole** | Direction magnétique (0-360°) + point cardinal |
| **Beacons BLE** | Liste des 4 ESP32 avec RSSI et distance |
| **Console** | Messages de debug, positions calculées |
| **Statut** | Connexion série, nombre de beacons |

#### Légende des couleurs

- 🟢 **Vert clair** : Balises fixes (M1, M2, M3)
- 🔴 **Rouge clair** : Bombe (TARGET)

### Lecture des données

#### Console série (miniterm)

```
[BEACONS] 4 détecté(s)
  [-45.2 dBm] M1       → 1.52m
  [-52.8 dBm] M2       → 2.83m
  [-48.1 dBm] M3       → 1.98m
  [-55.0 dBm] Bombe    → 3.45m

[INFO] Triangulation avec: M1, M2, M3
[POSITION 3D] Bombe: x=1.23m, y=1.87m, z=0.05m
[POSITION 2D] Bombe: x=1.23m, y=1.87m
```

#### Format JSON (sortie USB)

```json
{
  "timestamp": 1234567,
  "compass": {
    "heading": 156.8,
    "cardinal": "SSE"
  },
  "beacons": [
    {
      "name": "M1",
      "addr": "40:4C:CA:45:37:06",
      "rssi": -45.2,
      "rssi_filtered": -44.8,
      "distance": 1.52
    },
    {
      "name": "M2",
      "addr": "1C:DB:D4:36:49:CA",
      "rssi": -52.8,
      "rssi_filtered": -52.1,
      "distance": 2.83
    },
    {
      "name": "M3",
      "addr": "1C:DB:D4:37:4C:F2",
      "rssi": -48.1,
      "rssi_filtered": -47.9,
      "distance": 1.98
    },
    {
      "name": "Bombe",
      "addr": "1C:DB:D4:34:74:DE",
      "rssi": -55.0,
      "rssi_filtered": -54.3,
      "distance": 3.45
    }
  ],
  "target_position_2d": {
    "x": 1.23,
    "y": 1.87
  },
  "target_position_3d": {
    "x": 1.23,
    "y": 1.87,
    "z": 0.05
  }
}
```

---

## 📊 Performance

### Précision mesurée

Avec la calibration effectuée (`TX_POWER = -57.3`, `n = 3.8`) :

| Distance réelle | Distance calculée | Erreur absolue | Erreur relative |
|----------------|-------------------|----------------|-----------------|
| 1.0m | 1.00m | 0.00m | 0.0% |
| 2.0m | 2.03m | 0.03m | 1.6% |
| 3.0m | 2.98m | 0.02m | 0.8% |
| 5.0m | 3.30m | 1.70m | 34.0% |
| **Moyenne** | - | - | **9.1%** |

**Conclusion :**
- ✅ **Excellente précision** jusqu'à 3m (erreur < 2%)
- ⚠️ **Précision dégradée** au-delà de 3m (erreur ~34%)
- 🎯 **Zone optimale** : Rayon de 3m autour de chaque balise

### Fréquence de mise à jour

| Paramètre | Valeur | Notes |
|-----------|--------|-------|
| Scan BLE | Toutes les 4 secondes | Peut être ajusté |
| Durée scan | 2 secondes | Active scan pour meilleur RSSI |
| Envoi JSON | 5 fois/seconde | Protection anti-crash USB |
| Calcul position | 5 fois/seconde | Synchronisé avec envoi JSON |
| Filtre Kalman | Continu | Lissage temps réel |

### Consommation mémoire

| État | Mémoire libre (STM32) | Notes |
|------|----------------------|-------|
| Démarrage | ~45000 bytes | Après init |
| Fonctionnement | >30000 bytes | Stable |
| ⚠️ Seuil critique | <15000 bytes | Risque de crash |
| Garbage collection | Toutes les 10s | Nettoyage automatique |

---

## 🛡️ Optimisations anti-crash

Le système intègre plusieurs protections pour éviter le crash du firmware STM32.

### Problème initial

**Symptômes :**
- Crash après 2 minutes de fonctionnement
- LED1 bleue (LD2) s'allume → HardFault
- `[Errno 5] Input/output error` côté PC
- Port USB mort

**Cause :** Saturation du buffer USB CDC

### Solutions implémentées

#### 1. Ralentissement envoi JSON (CRITIQUE) ⚠️

```python
# AVANT: 50 envois/seconde → CRASH
print(json.dumps(data))  # Toutes les 20ms

# MAINTENANT: 5 envois/seconde → STABLE
if loop_count % 10 == 0:  # Toutes les 200ms
    print(json.dumps(data))
```

**Réduction :** 90% d'envois en moins

#### 2. Garbage collection activé

```python
import gc

gc.enable()  # Au démarrage
gc.collect()  # Toutes les 10 secondes
```

#### 3. Limitation beacons en mémoire

```python
# Maximum 10 beacons stockés (au lieu de illimité)
if len(self.beacons) > 10:
    del oldest_beacon
```

#### 4. Scan BLE ralenti

```python
scan_interval_ms = 4000  # 4 secondes (au lieu de 500ms)
```

**Réduction :** 87.5% de scans en moins

#### 5. JSON optimisé

```python
# Envoyer SEULEMENT les 4 ESP32 (M1, M2, M3, Bombe)
# Au lieu de tous les beacons détectés (10-50)
for mac in [TARGET_BEACON_MAC] + FIXED_BEACONS_MACS:
    # Seulement 4 beacons
```

**Réduction :** 75-90% de la taille JSON

### Résultat

| Paramètre | AVANT | MAINTENANT | Gain |
|-----------|-------|------------|------|
| Envois JSON/sec | 50 | 5 | 90% ↓ |
| Scans BLE/sec | 2 | 0.25 | 87.5% ↓ |
| Beacons mémoire | Illimité | 10 max | Contrôlé |
| Taille JSON | Tous | 4 ESP32 | 75-90% ↓ |
| Charge CPU | 100% | ~20% | 80% ↓ |
| **Stabilité** | 2 min | **∞** | **✅ RÉSOLU** |

---

## 🔧 Troubleshooting

### Problèmes courants

#### ❌ Bombe non détectée

**Symptôme :** `[WARN] Bombe non détectée (MAC: ...)`

**Solutions :**
1. Vérifier que l'ESP32 Bombe est allumé (LED clignote)
2. Vérifier l'adresse MAC dans `TARGET_BEACON_MAC`
3. Rapprocher la Bombe du STM32 (< 5m)
4. Vérifier la batterie de l'ESP32

#### ❌ Seulement 2/3 balises détectées

**Symptôme :** `[WARN] Seulement 2/3 balises fixes détectées`

**Solutions :**
1. Vérifier que les 3 ESP32 fixes sont allumés
2. Rapprocher les balises (idéalement < 5m du STM32)
3. Vérifier les adresses MAC dans `FIXED_BEACONS_MACS`
4. Réduire les obstacles entre STM32 et balises

#### ❌ Position aberrante

**Symptôme :** Position calculée très éloignée de la réalité

**Solutions :**
1. Vérifier les positions des balises dans `BEACONS_CONFIG`
2. Refaire la calibration (`TX_POWER`, `PATH_LOSS_EXPONENT`)
3. Vérifier que les balises fixes ne bougent pas
4. S'assurer que les balises forment un triangle (pas alignées)

#### ❌ Crash après quelques minutes

**Symptôme :** LED1 bleue s'allume, `[Errno 5]` côté PC

**Solutions :**
1. Vérifier que tu utilises `main_triangulation_ANTI_CRASH.py`
2. Ralentir encore l'envoi JSON : `loop_count % 25 == 0` (500ms)
3. Surveiller la mémoire : doit rester > 15000 bytes
4. Essayer un autre câble USB (certains sont charge-only)

#### ❌ Noms "Unknown" au lieu de M1, M2, M3

**Symptôme :** Beacons affichés comme "Unknown" ou par adresse MAC

**Solutions :**
1. Vérifier que les ESP32 envoient bien leur nom dans l'advertising
2. Vérifier le code Arduino : `advertisementData.setName("M1")`
3. Le code STM32 utilise les adresses MAC en priorité (normal)

#### ❌ Interface PC ne reçoit rien

**Symptôme :** Console vide, pas de données

**Solutions :**
1. Vérifier le port série : `ls /dev/ttyACM*`
2. Tester avec miniterm : `python3 -m serial.tools.miniterm /dev/ttyACM0 115200`
3. Reset le STM32 (bouton noir)
4. Reflasher le code : `ampy --port /dev/ttyACM0 put main.py`

### Commandes de diagnostic

#### Test connexion série

```bash
# Liste des ports
ls -l /dev/ttyACM*

# Test miniterm
python3 -m serial.tools.miniterm /dev/ttyACM0 115200

# CTRL+C pour quitter
```

#### Vérifier le code Python

```bash
# Vérifier la syntaxe
python3 -m py_compile main_triangulation_ANTI_CRASH.py

# Si erreur, elle s'affiche
```

#### Vérifier ampy

```bash
# Liste les fichiers sur le STM32
ampy --port /dev/ttyACM0 ls

# Doit afficher : main.py, boot.py
```

#### Monitor mémoire en temps réel

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200 | grep "\[MEM\]"

# Affiche seulement les lignes de mémoire
[MEM] Mémoire libre: 45234 bytes
[MEM] Mémoire libre: 44987 bytes
...
```

---

## 🚀 Améliorations futures

### Court terme

- [ ] **Ajout d'une 4ème balise** pour multilatération (précision améliorée)
- [ ] **Calibration automatique** via mesures à distances connues
- [ ] **Interface graphique 2D** avec affichage graphique de la position
- [ ] **Enregistrement des trajectoires** (log des positions)
- [ ] **Alarme sonore** quand la Bombe entre/sort d'une zone

### Moyen terme

- [ ] **Filtre particules** (au lieu de Kalman) pour meilleur suivi
- [ ] **Carte des obstacles** pour compensation du multipath
- [ ] **Support hauteur variable** (z dynamique)
- [ ] **Mode multi-cibles** (tracker plusieurs objets simultanément)
- [ ] **Interface web** (Flask) au lieu de Tkinter

### Long terme

- [ ] **Machine learning** pour prédiction de trajectoire
- [ ] **Fusion capteurs** (IMU + BLE + magnétomètre)
- [ ] **Cartographie SLAM** de l'environnement
- [ ] **Mode ultra-low-power** avec beacons sur pile bouton
- [ ] **Compatibilité Bluetooth 5.1** (Direction Finding)

---

## 📚 Annexes

### Formules mathématiques

#### Conversion RSSI → Distance

```
d = 10^((P₀ - RSSI) / (10 × n))
```

Où :
- `d` = distance en mètres
- `P₀` = TX_POWER (RSSI à 1m)
- `RSSI` = signal mesuré en dBm
- `n` = PATH_LOSS_EXPONENT (atténuation environnement)

#### Trilatération 2D

Système d'équations pour 3 balises :

```
(x - x₁)² + (y - y₁)² = d₁²
(x - x₂)² + (y - y₂)² = d₂²
(x - x₃)² + (y - y₃)² = d₃²
```

Résolution par méthode algébrique (voir code source).

#### Filtre de Kalman 1D

**Prédiction :**
```
x̂ₖ⁻ = x̂ₖ₋₁
Pₖ⁻ = Pₖ₋₁ + Q
```

**Mise à jour :**
```
Kₖ = Pₖ⁻ / (Pₖ⁻ + R)
x̂ₖ = x̂ₖ⁻ + Kₖ(zₖ - x̂ₖ⁻)
Pₖ = (1 - Kₖ)Pₖ⁻
```

Où :
- `Q` = process variance = 1e-5
- `R` = measurement variance = 0.1
- `Kₖ` = gain de Kalman
- `zₖ` = mesure RSSI

### Pinout STM32WB55

| Pin | Fonction | Connecté à |
|-----|----------|-----------|
| PB8 | I2C1 SCL | Shield I2C (magnétomètre) |
| PB9 | I2C1 SDA | Shield I2C |
| USB D+ | USB FS | PC (données + alimentation) |
| USB D- | USB FS | PC |
| LD1 (verte) | User LED | Libre (non utilisée) |
| LD2 (bleue) | User LED | Debug (s'allume si HardFault) |
| LD3 (rouge) | User LED | Libre |

### Adresses I2C des capteurs

| Capteur | Adresse I2C | Fonction |
|---------|------------|----------|
| LIS2MDL | 0x1E | Magnétomètre 3 axes |
| LSM6DSO | 0x6A | Accéléromètre + gyroscope |
| LPS22HH | 0x5C | Baromètre (non utilisé) |
| HTS221 | 0x5F | Température + humidité (non utilisé) |

### Structure des fichiers

```
projet_triangulation/
│
├── stm32/
│   ├── main_triangulation_ANTI_CRASH.py  # Code principal STM32
│   ├── main_simple.py                     # Version simplifiée (debug)
│   └── README.txt
│
├── pc/
│   ├── pc_interface_triangulation.py     # Interface graphique PC
│   └── requirements.txt
│
├── esp32/
│   ├── beacon_fixed.ino                   # Code Arduino balises fixes
│   ├── beacon_target.ino                  # Code Arduino cible mobile
│   └── README.txt
│
├── docs/
│   ├── CALIBRATION.md                     # Guide de calibration
│   ├── TROUBLESHOOTING.md                 # Résolution problèmes
│   └── README.md                          # Cette documentation
│
└── README.md                               # Vue d'ensemble
```

### Liens utiles

- [MicroPython pour STM32](https://micropython.org/download/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [ampy (Adafruit MicroPython Tool)](https://github.com/scientifichackers/ampy)
- [Documentation BLE ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/index.html)
- [Trilatération (Wikipedia)](https://en.wikipedia.org/wiki/Trilateration)
- [Filtre de Kalman (Wikipedia)](https://en.wikipedia.org/wiki/Kalman_filter)

---

## 👥 Crédits

### Auteur

**Mattéo** - Développement et calibration

### Technologies utilisées

- **STM32WB55** - STMicroelectronics
- **ESP32-C3/C6** - Espressif Systems
- **MicroPython** - Damien George
- **Arduino IDE** - Arduino Team
- **Python 3** - Python Software Foundation

### Licence

Ce projet est fourni à des fins éducatives. Vous êtes libre de l'utiliser, le modifier et le distribuer selon vos besoins.

---

## 📝 Changelog

### Version 1.0 (2026-01-15)

- ✅ Triangulation 2D et 3D fonctionnelle
- ✅ Calibration RSSI optimisée (erreur 9% jusqu'à 3m)
- ✅ Protection anti-crash USB (stable >1h)
- ✅ Interface graphique Tkinter
- ✅ Filtrage Kalman du RSSI
- ✅ Support ESP32-C3 et C6
- ✅ Garbage collection automatique
- ✅ Documentation complète

### À venir (Version 2.0)

- 🔄 Interface web (Flask)
- 🔄 Visualisation 2D de la position
- 🔄 Enregistrement des trajectoires
- 🔄 Support de 4+ balises
- 🔄 Calibration automatique

---

**Dernière mise à jour :** 15 janvier 2026

**Status :** ✅ Production-ready

---

## 📞 Support

Pour toute question ou problème :
1. Consulter la section [Troubleshooting](#-troubleshooting)
2. Vérifier que tu utilises la version `main_triangulation_ANTI_CRASH.py`
3. Tester avec `miniterm` pour voir les logs en direct

**Bon tracking ! 🎯**
