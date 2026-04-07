"""
STM32WB55 - Triangulation BLE avec Beacons Cypress
Scanne 3 beacons fixes + 1 beacon mobile, calcule la position

Matériel:
- STM32WB55 Nucleo
- Shield environnemental (magnétomètre)
- 4 Beacons Cypress BLE
"""

import time
import json
import machine
from machine import Pin, I2C
import ubluetooth
import struct
import math
import gc  # Garbage collector pour éviter les fuites mémoire

# ============================================================================
# CONFIGURATION I2C
# ============================================================================
I2C_BUS = 1

# Adresses I2C
LIS2MDL_ADDR = 0x1E
LSM6DSO_ADDR = 0x6A

# Registres LIS2MDL
LIS2MDL_CFG_REG_A = 0x60
LIS2MDL_CFG_REG_B = 0x61
LIS2MDL_CFG_REG_C = 0x62
LIS2MDL_OUTX_L = 0x68

# Registres LSM6DSO
LSM6DSO_WHO_AM_I = 0x0F
LSM6DSO_CTRL1_XL = 0x10
LSM6DSO_OUTX_L_A = 0x28

# ============================================================================
# CONFIGURATION BEACONS
# ============================================================================
# IMPORTANT: Mesurer précisément les positions (x, y, z) en mètres !
# x, y = position au sol, z = hauteur par rapport au sol

# ADRESSES MAC DES ESP32
BEACONS_CONFIG = {
    # Balise fixe M1 (ESP32-C6 - utilisé pour calibration) - ORIGINE
    "40:4C:CA:45:37:06": {
        "position": (0.0, 0.0, 0.0),  # Origine au sol
        "role": "fixed",
        "name": "M1"
    },
    # Balise fixe M2 (ESP32-C3) - Sur l'axe Y à 3m
    "1C:DB:D4:36:49:CA": {
        "position": (0.10, 3.0, 0.0),  # 10cm décalé en X, 3m en Y, au sol
        "role": "fixed",
        "name": "M2"
    },
    # Balise fixe M3 (ESP32-C3) - Diagonal
    "1C:DB:D4:37:4C:F2": {
        "position": (2.70, 2.70, 0.0),  # 2.70m en X et Y, au sol
        "role": "fixed",
        "name": "M3"
    },
    # Beacon BOMBE (ESP32-C3) - Objet à localiser
    "1C:DB:D4:34:74:DE": {
        "position": None,  # Position à calculer par triangulation
        "role": "target",
        "name": "Bombe"
    }
}

# Liste des adresses MAC des balises fixes (dans l'ordre de priorité)
FIXED_BEACONS_MACS = ["40:4C:CA:45:37:06", "1C:DB:D4:36:49:CA", "1C:DB:D4:37:4C:F2"]

# Adresse MAC du beacon mobile à tracker (la BOMBE)
TARGET_BEACON_MAC = "1C:DB:D4:34:74:DE"

# MODE DEBUG: Afficher TOUS les beacons détectés
DEBUG_SHOW_ALL_BEACONS = True  # Mettre False une fois que tout fonctionne

# Paramètres de conversion RSSI -> Distance (CALIBRÉS )
TX_POWER = -57.3  # Puissance mesurée à 1m avec M1 (ESP32-C6)
PATH_LOSS_EXPONENT = 3.8  # Calibré pour environnement intérieur (erreur ~9% jusqu'à 3m)

# ============================================================================
# CLASSE FILTRE KALMAN (pour lisser le RSSI)
# ============================================================================
class KalmanFilter:
    """Filtre de Kalman 1D pour lisser le RSSI"""
    
    def __init__(self, process_variance=1e-5, measurement_variance=0.1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_value = None
        self.error_covariance = 1.0
    
    def update(self, measurement):
        if self.estimated_value is None:
            self.estimated_value = measurement
            return measurement
        
        # Prédiction
        prediction = self.estimated_value
        prediction_cov = self.error_covariance + self.process_variance
        
        # Mise à jour
        kalman_gain = prediction_cov / (prediction_cov + self.measurement_variance)
        self.estimated_value = prediction + kalman_gain * (measurement - prediction)
        self.error_covariance = (1 - kalman_gain) * prediction_cov
        
        return self.estimated_value


# ============================================================================
# CLASSE BLE SCANNER AVANCÉ
# ============================================================================
class BLEBeaconScanner:
    """Scanner BLE spécialisé pour les beacons"""
    
    def __init__(self):
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._ble_irq)
        
        # Dictionnaire des beacons détectés
        self.beacons = {}
        
        # Filtres Kalman pour chaque beacon
        self.filters = {}
        
        self.scanning = False
        print("[BLE] Scanner initialisé pour beacons")
    
    def _ble_irq(self, event, data):
        """Callback IRQ BLE"""
        if event == 5:  # _IRQ_SCAN_RESULT
            addr_type, addr, adv_type, rssi, adv_data = data
            addr_str = ':'.join(['%02X' % b for b in bytes(addr)])
            
            # Essayer plusieurs méthodes pour identifier le beacon
            name = self._parse_adv_name(adv_data)
            
            # Si pas de nom, essayer de parser iBeacon
            if not name:
                ibeacon_data = self._parse_ibeacon(adv_data)
                if ibeacon_data:
                    # Utiliser Major comme identifiant
                    major = ibeacon_data['major']
                    name = "BEACON_%d" % major
            
            # Si toujours pas de nom, utiliser l'adresse MAC
            if not name:
                name = "MAC_%s" % addr_str[-8:].replace(':', '')
            
            # DEBUG: Afficher les données brutes toutes les 100 détections
            if not hasattr(self, '_debug_count'):
                self._debug_count = 0
            
            self._debug_count += 1
            if self._debug_count % 100 == 0:
                print("[DEBUG] Adv data: %s" % ' '.join(['%02X' % b for b in adv_data[:20]]))
                print("[DEBUG] Name: %s, Addr: %s" % (name, addr_str))
            
            # Vérifier si c'est un beacon configuré
            if self._is_target_beacon(name, addr_str):
                # Créer un filtre si nécessaire
                if name not in self.filters:
                    self.filters[name] = KalmanFilter()
                
                # Filtrer le RSSI
                filtered_rssi = self.filters[name].update(rssi)
                
                self.beacons[name] = {
                    "name": name,
                    "addr": addr_str,
                    "rssi": rssi,
                    "rssi_filtered": filtered_rssi,
                    "last_seen": time.ticks_ms()
                }
                
                if len(self.beacons) > 10:
                    oldest_name = min(self.beacons.items(), key=lambda x: x[1]["last_seen"])[0]
                    del self.beacons[oldest_name]
                    if oldest_name in self.filters:
                        del self.filters[oldest_name]
        
        elif event == 6:  # _IRQ_SCAN_DONE
            self.scanning = False
    
    def _parse_ibeacon(self, adv_data):
        """
        Parse les données iBeacon (Apple)
        Format: Company ID (0x004C) + Type (0x02) + Length (0x15) + UUID + Major + Minor + TxPower
        """
        try:
            i = 0
            while i < len(adv_data):
                length = adv_data[i]
                if length == 0:
                    break
                
                ad_type = adv_data[i + 1]
                
                # 0xFF = Manufacturer Specific Data
                if ad_type == 0xFF and length >= 25:
                    # Vérifier Company ID Apple (0x004C)
                    company_id = (adv_data[i + 3] << 8) | adv_data[i + 2]
                    if company_id == 0x004C:
                        # Type iBeacon (0x02)
                        if adv_data[i + 4] == 0x02 and adv_data[i + 5] == 0x15:
                            # Extraire UUID (16 bytes)
                            uuid_bytes = adv_data[i + 6:i + 22]
                            # Extraire Major (2 bytes, big endian)
                            major = (adv_data[i + 22] << 8) | adv_data[i + 23]
                            # Extraire Minor (2 bytes, big endian)
                            minor = (adv_data[i + 24] << 8) | adv_data[i + 25]
                            
                            return {
                                "uuid": uuid_bytes,
                                "major": major,
                                "minor": minor
                            }
                
                i += length + 1
        except:
            pass
        
        return None
    
    def _is_target_beacon(self, name, addr):
        """Vérifie si c'est un beacon qui nous intéresse (par adresse MAC)"""
        # Vérifier par adresse MAC (méthode principale)
        if addr in BEACONS_CONFIG:
            return True
        
        # Vérifier par nom aussi (si configuré dans le code ESP32)
        for mac, config in BEACONS_CONFIG.items():
            if config.get("name") == name:
                return True
        
        # MODE DEBUG: Si activé, accepter tous les beacons
        if DEBUG_SHOW_ALL_BEACONS:
            return True
        
        return False
    
    def _parse_adv_name(self, adv_data):
        """Parse le nom du beacon"""
        try:
            i = 0
            while i < len(adv_data):
                length = adv_data[i]
                if length == 0:
                    break
                
                ad_type = adv_data[i + 1]
                
                # 0x09 = Complete Local Name, 0x08 = Shortened Local Name
                if ad_type in (0x08, 0x09):
                    name_bytes = adv_data[i + 2:i + 1 + length]
                    return name_bytes.decode('utf-8', 'ignore')
                
                i += length + 1
        except:
            pass
        
        return None
    
    def start_scan(self, duration_ms=1000):
        """Démarre un scan"""
        if not self.scanning:
            self.scanning = True
            self.ble.gap_scan(duration_ms, 16000, 16000, True)
    
    def get_beacons(self, max_age_ms=5000):
        """Retourne les beacons actifs"""
        current_time = time.ticks_ms()
        active_beacons = {}
        
        for name, info in self.beacons.items():
            if time.ticks_diff(current_time, info["last_seen"]) < max_age_ms:
                active_beacons[name] = info
        
        return active_beacons


# ============================================================================
# FONCTIONS DE TRIANGULATION
# ============================================================================
def rssi_to_distance(rssi, tx_power=TX_POWER, n=PATH_LOSS_EXPONENT):
    """
    Convertit RSSI en distance (mètres)
    Formule: d = 10^((TxPower - RSSI) / (10*n))
    """
    if rssi == 0 or rssi > 0:
        return -1.0
    
    ratio = (tx_power - rssi) / (10.0 * n)
    distance = pow(10, ratio)
    
    return distance


def trilateration_3d(beacons_pos, distances):
    """
    Calcule la position 3D par trilatération
    Gère les cas où les beacons sont à différentes hauteurs
    
    beacons_pos: [(x1,y1,z1), (x2,y2,z2), (x3,y3,z3)]
    distances: [d1, d2, d3]
    
    Returns: (x, y, z) ou None si échec
    """
    if len(beacons_pos) < 3 or len(distances) < 3:
        return None
    
    try:
        # Positions des 3 premières balises
        p1 = beacons_pos[0]
        p2 = beacons_pos[1]
        p3 = beacons_pos[2]
        
        r1, r2, r3 = distances[0], distances[1], distances[2]
        
        # Vecteurs
        ex = [(p2[i] - p1[i]) for i in range(3)]
        # Normaliser ex
        d = (ex[0]**2 + ex[1]**2 + ex[2]**2) ** 0.5
        if d < 0.0001:
            return None
        ex = [ex[i]/d for i in range(3)]
        
        # Vecteur i
        i_val = sum([ex[k] * (p3[k] - p1[k]) for k in range(3)])
        
        # Vecteur ey
        ey = [(p3[k] - p1[k] - i_val * ex[k]) for k in range(3)]
        # Normaliser ey
        d = (ey[0]**2 + ey[1]**2 + ey[2]**2) ** 0.5
        if d < 0.0001:
            return None
        ey = [ey[i]/d for i in range(3)]
        
        # Vecteur ez (produit vectoriel)
        ez = [
            ex[1]*ey[2] - ex[2]*ey[1],
            ex[2]*ey[0] - ex[0]*ey[2],
            ex[0]*ey[1] - ex[1]*ey[0]
        ]
        
        # Distances dans le nouveau repère
        d_12 = (sum([(p2[k] - p1[k])**2 for k in range(3)])) ** 0.5
        j = sum([ey[k] * (p3[k] - p1[k]) for k in range(3)])
        
        # Calcul de x, y, z
        x = (r1**2 - r2**2 + d_12**2) / (2 * d_12)
        y = (r1**2 - r3**2 + i_val**2 + j**2) / (2 * j) - (i_val * x) / j
        
        # z peut avoir 2 solutions (sphère)
        z_squared = r1**2 - x**2 - y**2
        if z_squared < 0:
            z = 0  # Pas de solution, on met z=0
        else:
            z = z_squared ** 0.5
        
        # Position finale dans le repère d'origine
        pos_x = p1[0] + x * ex[0] + y * ey[0] + z * ez[0]
        pos_y = p1[1] + x * ex[1] + y * ey[1] + z * ez[1]
        pos_z = p1[2] + x * ex[2] + y * ey[2] + z * ez[2]
        
        return (pos_x, pos_y, pos_z)
    
    except Exception as e:
        print("[WARN] Erreur trilatération 3D: %s" % e)
        return None


def trilateration_2d(beacons_pos, distances):
    """
    Calcule la position 2D par trilatération (projection au sol)
    Utilise une méthode algébrique simplifiée
    
    beacons_pos: [(x1,y1,z1), (x2,y2,z2), (x3,y3,z3)]
    distances: [d1, d2, d3]
    
    Returns: (x, y) ou None si échec
    """
    if len(beacons_pos) < 3 or len(distances) < 3:
        return None
    
    try:
        # Extraire seulement x, y (ignorer z)
        x1, y1 = beacons_pos[0][0], beacons_pos[0][1]
        x2, y2 = beacons_pos[1][0], beacons_pos[1][1]
        x3, y3 = beacons_pos[2][0], beacons_pos[2][1]
        
        # Ajuster les distances pour tenir compte de la différence de hauteur
        z1, z2, z3 = beacons_pos[0][2], beacons_pos[1][2], beacons_pos[2][2]
        z_target = (z1 + z2 + z3) / 3  # Estimation hauteur cible
        
        # Distance au sol = sqrt(distance_3d² - delta_z²)
        d1 = (distances[0]**2 - (z1 - z_target)**2) ** 0.5 if distances[0]**2 > (z1 - z_target)**2 else distances[0]
        d2 = (distances[1]**2 - (z2 - z_target)**2) ** 0.5 if distances[1]**2 > (z2 - z_target)**2 else distances[1]
        d3 = (distances[2]**2 - (z3 - z_target)**2) ** 0.5 if distances[2]**2 > (z3 - z_target)**2 else distances[2]
        
        # Méthode algébrique
        A = 2 * (x2 - x1)
        B = 2 * (y2 - y1)
        C = d1*d1 - d2*d2 - x1*x1 + x2*x2 - y1*y1 + y2*y2
        
        D = 2 * (x3 - x2)
        E = 2 * (y3 - y2)
        F = d2*d2 - d3*d3 - x2*x2 + x3*x3 - y2*y2 + y3*y3
        
        # Résolution
        denom = A * E - B * D
        if abs(denom) < 0.0001:
            return None
        
        x = (C * E - F * B) / denom
        y = (C * D - A * F) / denom
        
        return (x, y)
    
    except Exception as e:
        print("[WARN] Erreur trilatération 2D: %s" % e)
        return None


# ============================================================================
# CLASSE MAGNÉTOMÈTRE (inchangée)
# ============================================================================
class LIS2MDL:
    def __init__(self, i2c, address=LIS2MDL_ADDR):
        self.i2c = i2c
        self.address = address
        if address not in self.i2c.scan():
            raise RuntimeError("[MAG] LIS2MDL non trouvé")
        self._init_sensor()
        print("[MAG] LIS2MDL initialisé")
    
    def _init_sensor(self):
        self.i2c.writeto_mem(self.address, LIS2MDL_CFG_REG_A, b'\x00')
        self.i2c.writeto_mem(self.address, LIS2MDL_CFG_REG_B, b'\x02')
        self.i2c.writeto_mem(self.address, LIS2MDL_CFG_REG_C, b'\x10')
        time.sleep_ms(20)
    
    def read_raw(self):
        data = self.i2c.readfrom_mem(self.address, LIS2MDL_OUTX_L, 6)
        x = struct.unpack('<h', data[0:2])[0]
        y = struct.unpack('<h', data[2:4])[0]
        z = struct.unpack('<h', data[4:6])[0]
        return x, y, z
    
    def read_gauss(self):
        x, y, z = self.read_raw()
        scale = 1.5 / 1000.0
        return x * scale, y * scale, z * scale
    
    def calculate_heading(self):
        x, y, z = self.read_gauss()
        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360
        return heading_deg


# ============================================================================
# CLASSE ACCÉLÉROMÈTRE
# ============================================================================
class LSM6DSO:
    def __init__(self, i2c, address=LSM6DSO_ADDR):
        self.i2c = i2c
        self.address = address
        if address not in self.i2c.scan():
            raise RuntimeError("[ACCEL] LSM6DSO non trouvé")
        self._init_sensor()
        print("[ACCEL] LSM6DSO initialisé")
    
    def _init_sensor(self):
        self.i2c.writeto_mem(self.address, LSM6DSO_CTRL1_XL, b'\x30')
        time.sleep_ms(20)
    
    def read_raw(self):
        data = self.i2c.readfrom_mem(self.address, LSM6DSO_OUTX_L_A, 6)
        x = struct.unpack('<h', data[0:2])[0]
        y = struct.unpack('<h', data[2:4])[0]
        z = struct.unpack('<h', data[4:6])[0]
        return x, y, z
    
    def read_g(self):
        x, y, z = self.read_raw()
        scale = 0.000061
        return x * scale, y * scale, z * scale


def get_cardinal_direction(angle):
    directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                  "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"]
    index = int((angle + 11.25) / 22.5) % 16
    return directions[index]


# ============================================================================
# BOUCLE PRINCIPALE
# ============================================================================
def main():
    print("=" * 60)
    print("STM32WB55 - TRIANGULATION BLE + BOUSSOLE")
    print("=" * 60)
    
    # Init I2C
    try:
        i2c = I2C(I2C_BUS, freq=400000)
        print("[I2C] Initialisé sur bus %d" % I2C_BUS)
    except Exception as e:
        print("[ERREUR] Init I2C: %s" % e)
        return
    
    # Init capteurs
    try:
        mag = LIS2MDL(i2c)
    except:
        mag = None
    
    try:
        accel = LSM6DSO(i2c)
    except:
        accel = None
    
    # Init scanner BLE
    try:
        ble_scanner = BLEBeaconScanner()
    except Exception as e:
        print("[ERREUR] Init BLE: %s" % e)
        return
    
    print("\n[INFO] Démarrage de l'acquisition...")
    print("[INFO] Recherche des beacons ESP32...\n")
    
    # Activer le garbage collector
    gc.enable()
    print("[MEM] Garbage collector activé")
    print("[MEM] Mémoire libre au démarrage: %d bytes" % gc.mem_free())
    
    loop_count = 0
    last_scan_time = time.ticks_ms()
    scan_interval_ms = 4000 
    scan_duration_ms = 2000  # Durée de chaque scan: 2 secondes
    
    last_gc_time = time.ticks_ms()
    gc_interval_ms = 10000  # Garbage collection toutes les 10 secondes
    
    while True:
        try:
            current_time = time.ticks_ms()
            
            # ========================================================
            # GARBAGE COLLECTION PÉRIODIQUE
            # ========================================================
            if time.ticks_diff(current_time, last_gc_time) >= gc_interval_ms:
                gc.collect()
                last_gc_time = current_time
                if loop_count % 500 == 0:  # Afficher toutes les 10 secondes
                    print("[MEM] Mémoire libre: %d bytes" % gc.mem_free())
            
            # ========================================================
            # SCAN BLE PÉRIODIQUE (RALENTI)
            # ========================================================
            if time.ticks_diff(current_time, last_scan_time) >= scan_interval_ms:
                if not ble_scanner.scanning:
                    ble_scanner.start_scan(duration_ms=scan_duration_ms)
                    last_scan_time = current_time
            
            # Lecture magnétomètre
            if mag:
                try:
                    heading = mag.calculate_heading()
                    cardinal = get_cardinal_direction(heading)
                except:
                    heading = 0
                    cardinal = "N"
            else:
                heading = 0
                cardinal = "N"
            
            # Lecture accéléromètre
            if accel:
                try:
                    accel_x, accel_y, accel_z = accel.read_g()
                except:
                    accel_x = accel_y = 0.0
                    accel_z = 1.0
            else:
                accel_x = accel_y = 0.0
                accel_z = 1.0
            
            # Récupérer les beacons
            beacons = ble_scanner.get_beacons(max_age_ms=2000)
            
            # MODE DEBUG: Afficher tous les beacons si activé
            if DEBUG_SHOW_ALL_BEACONS and loop_count % 50 == 0:
                all_beacons = ble_scanner.beacons  # Tous les beacons, pas juste les filtrés
                if all_beacons:
                    print("\n" + "="*60)
                    print("[DEBUG] TOUS LES BEACONS (%d détectés)" % len(all_beacons))
                    print("="*60)
                    for name, info in all_beacons.items():
                        print("  [%5.1f dBm] %-20s %s" % (info['rssi'], name, info['addr']))
                    print("="*60 + "\n")
            
            # Affichage beacons configurés
            if loop_count % 50 == 0 and beacons:
                print("\n" + "="*60)
                print("[BEACONS] %d beacon(s) configuré(s) détecté(s)" % len(beacons))
                print("="*60)
                
                for name, info in beacons.items():
                    rssi_filtered = info['rssi_filtered']
                    distance = rssi_to_distance(rssi_filtered)
                    print("  [%5.1f dBm] %-12s → %.2fm  %s" % (rssi_filtered, name, distance, info['addr']))
                print("="*60 + "\n")
            
            # Calcul de position si on a 3+ balises fixes + la cible
            target_position = None
            target_position_3d = None
            
            if TARGET_BEACON_MAC in beacons:
                # Récupérer les balises fixes configurées
                fixed_beacons = []
                distances = []
                detected_beacons = []
                
                # Parcourir les balises fixes dans l'ordre de priorité
                for beacon_mac in FIXED_BEACONS_MACS:
                    if beacon_mac in beacons:
                        pos = BEACONS_CONFIG[beacon_mac]["position"]
                        rssi = beacons[beacon_mac]["rssi_filtered"]
                        dist = rssi_to_distance(rssi)
                        
                        fixed_beacons.append(pos)
                        distances.append(dist)
                        detected_beacons.append(BEACONS_CONFIG[beacon_mac]["name"])
                
                # Trilatération si on a au moins 3 balises
                if len(fixed_beacons) >= 3:
                    # Utiliser les 3 premières balises
                    used_beacons = fixed_beacons[:3]
                    used_distances = distances[:3]
                    
                    # Essayer la 3D d'abord
                    target_position_3d = trilateration_3d(used_beacons, used_distances)
                    
                    # Aussi calculer la projection 2D
                    target_position = trilateration_2d(used_beacons, used_distances)
                    
                    if loop_count % 50 == 0:
                        print("[INFO] Triangulation avec: %s" % ', '.join(detected_beacons[:3]))
                        if target_position_3d:
                            print("[POSITION 3D] Bombe: x=%.2fm, y=%.2fm, z=%.2fm" % (target_position_3d[0], target_position_3d[1], target_position_3d[2]))
                        if target_position:
                            print("[POSITION 2D] Bombe: x=%.2fm, y=%.2fm" % (target_position[0], target_position[1]))
                            print("")
                
                elif len(fixed_beacons) < 3 and loop_count % 50 == 0:
                    print("[WARN] Seulement %d/3 balises fixes détectées: %s" % (len(fixed_beacons), detected_beacons))
                    print("[WARN] Impossible de trianguler (besoin de 3 minimum)")
                    print("")
            
            elif loop_count % 100 == 0:
                print("[WARN] Bombe non détectée (MAC: %s)" % TARGET_BEACON_MAC)
            
            # Construction du JSON (OPTIMISÉ - seulement les 4 ESP32)
            beacon_list = []
            # Parcourir SEULEMENT nos 4 ESP32 configurés
            for mac in [TARGET_BEACON_MAC] + FIXED_BEACONS_MACS:
                # Chercher par MAC d'abord
                if mac in beacons:
                    info = beacons[mac]
                    beacon_list.append({
                        "name": BEACONS_CONFIG[mac]["name"],
                        "addr": info["addr"],
                        "rssi": round(info["rssi"], 1),
                        "rssi_filtered": round(info["rssi_filtered"], 1),
                        "distance": round(rssi_to_distance(info["rssi_filtered"]), 2)
                    })
                else:
                    # Chercher par nom en fallback
                    for name, info in beacons.items():
                        if info["addr"] == mac:
                            beacon_list.append({
                                "name": BEACONS_CONFIG[mac]["name"],
                                "addr": info["addr"],
                                "rssi": round(info["rssi"], 1),
                                "rssi_filtered": round(info["rssi_filtered"], 1),
                                "distance": round(rssi_to_distance(info["rssi_filtered"]), 2)
                            })
                            break
            
            data_packet = {
                "timestamp": time.ticks_ms(),
                "compass": {
                    "heading": round(heading, 1),
                    "cardinal": cardinal
                },
                "beacons": beacon_list,
                "target_position_2d": {
                    "x": round(target_position[0], 2) if target_position else None,
                    "y": round(target_position[1], 2) if target_position else None
                } if target_position else None,
                "target_position_3d": {
                    "x": round(target_position_3d[0], 2) if target_position_3d else None,
                    "y": round(target_position_3d[1], 2) if target_position_3d else None,
                    "z": round(target_position_3d[2], 2) if target_position_3d else None
                } if target_position_3d else None,
                "loop": loop_count }
            # ========================================================
            # ENVOI JSON VIA USB SÉRIE
            # ========================================================
            # IMPORTANT: Envoyer seulement toutes les 200ms pour éviter
            # la saturation du buffer USB CDC (cause de crash firmware)
            # 
            # Options de fréquence d'envoi:
            # - loop_count % 10 == 0  → 200ms (5 fois/sec)  ← ACTUEL
            # - loop_count % 25 == 0  → 500ms (2 fois/sec)  ← Si crash persiste
            # - loop_count % 50 == 0  → 1000ms (1 fois/sec) ← Mode ultra-safe
            if loop_count % 10 == 0:
                json_str = json.dumps(data_packet)
                print(json_str)
            
            loop_count += 1
            time.sleep_ms(20)  # Boucle à ~50 Hz
        
        except KeyboardInterrupt:
            print("\n[INFO] Arrêt demandé")
            ble_scanner.ble.gap_scan(None)
            break
        
        except MemoryError:
            print("[ERREUR CRITIQUE] Plus de mémoire disponible!")
            gc.collect()
            print("[MEM] Après GC forcé: %d bytes" % gc.mem_free())
            time.sleep_ms(1000)
        
        except Exception as e:
            print("[ERREUR] Boucle: %s" % e)
            time.sleep_ms(100)


if __name__ == "__main__":
    main()