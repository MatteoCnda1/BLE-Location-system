#!/usr/bin/env python3
"""
Interface PC pour STM32 Boussole + BLE
Version avec console de debug intégrée
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import json
import threading
import queue
import time
import math

# ============================================================================
# CONFIGURATION - FILTRAGE DES BEACONS
# ============================================================================
# Liste des adresses MAC des ESP32 à afficher (autres = ignorés)
ALLOWED_MACS = [
    "40:4C:CA:45:37:06",  # M1 (ESP32-C6)
    "1C:DB:D4:36:49:CA",  # M2 (ESP32-C3)
    "1C:DB:D4:37:4C:F2",  # M3 (ESP32-C3)
    "1C:DB:D4:34:74:DE",  # Bombe (ESP32-C3)
]

# Noms des beacons (pour affichage)
BEACON_NAMES = {
    "40:4C:CA:45:37:06": "M1",
    "1C:DB:D4:36:49:CA": "M2",
    "1C:DB:D4:37:4C:F2": "M3",
    "1C:DB:D4:34:74:DE": "Bombe",
}
# ============================================================================

class CompassBLEInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Triangulation BLE + Boussole")
        self.root.geometry("1200x800")
        
        # Variables
        self.serial_port = None
        self.running = False
        self.data_queue = queue.Queue()
        self.latest_data = {}
        
        # Créer l'interface
        self._create_ui()
        
        # Démarrer la mise à jour périodique
        self._update()
    
    def _create_ui(self):
        """Crée l'interface utilisateur"""
        
        # Frame du haut : Connexion
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        
        ttk.Label(top_frame, text="Port série:").pack(side=tk.LEFT, padx=5)
        
        self.port_combo = ttk.Combobox(top_frame, width=20)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self._update_ports()
        
        ttk.Button(top_frame, text="🔄 Rafraîchir", command=self._update_ports).pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(top_frame, text="Connecter", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.status_label = ttk.Label(top_frame, text="❌ Déconnecté", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame principal : Séparé en deux colonnes
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Colonne gauche : Boussole + Données
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Canvas pour la boussole
        compass_frame = ttk.LabelFrame(left_frame, text="🧭 Boussole", padding=10)
        compass_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.compass_canvas = tk.Canvas(compass_frame, width=300, height=300, bg='white')
        self.compass_canvas.pack()
        
        # Infos boussole
        self.heading_label = ttk.Label(compass_frame, text="Cap: --- °", font=("Arial", 18, "bold"))
        self.heading_label.pack(pady=5)
        
        self.cardinal_label = ttk.Label(compass_frame, text="Direction: ---", font=("Arial", 14))
        self.cardinal_label.pack(pady=5)
        
        # Données capteurs
        data_frame = ttk.LabelFrame(left_frame, text="📊 Données Capteurs", padding=10)
        data_frame.pack(fill=tk.X, pady=5)
        
        self.mag_label = ttk.Label(data_frame, text="Magnétomètre: --- µT", font=("Courier", 10))
        self.mag_label.pack(anchor=tk.W, pady=2)
        
        self.accel_label = ttk.Label(data_frame, text="Accéléromètre: --- g", font=("Courier", 10))
        self.accel_label.pack(anchor=tk.W, pady=2)
        
        # Colonne droite : BLE + Console
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # Liste BLE
        ble_frame = ttk.LabelFrame(right_frame, text="📡 Périphériques BLE", padding=10)
        ble_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Treeview pour les périphériques BLE
        columns = ("name", "addr", "rssi", "distance")
        self.ble_tree = ttk.Treeview(ble_frame, columns=columns, show="headings", height=8)
        self.ble_tree.heading("name", text="Nom")
        self.ble_tree.heading("addr", text="Adresse MAC")
        self.ble_tree.heading("rssi", text="RSSI (dBm)")
        self.ble_tree.heading("distance", text="Distance")
        
        self.ble_tree.column("name", width=120)
        self.ble_tree.column("addr", width=130)
        self.ble_tree.column("rssi", width=80)
        self.ble_tree.column("distance", width=80)
        
        ble_scroll = ttk.Scrollbar(ble_frame, orient=tk.VERTICAL, command=self.ble_tree.yview)
        self.ble_tree.configure(yscrollcommand=ble_scroll.set)
        
        self.ble_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ble_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.ble_count_label = ttk.Label(ble_frame, text="0 périphériques détectés")
        self.ble_count_label.pack(pady=5)
        
        # Console de debug
        console_frame = ttk.LabelFrame(right_frame, text="🖥️ Console Debug", padding=10)
        console_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.console = scrolledtext.ScrolledText(console_frame, height=15, width=60, font=("Courier", 9), bg="#1e1e1e", fg="#00ff00")
        self.console.pack(fill=tk.BOTH, expand=True)
        
        # Ajouter un message d'accueil
        self._log_console("="*60)
        self._log_console("STM32 BOUSSOLE + BLE - Interface PC")
        self._log_console("="*60)
        self._log_console("En attente de connexion...\n")
    
    def _log_console(self, message):
        """Ajoute un message dans la console"""
        self.console.insert(tk.END, message + "\n")
        self.console.see(tk.END)
    
    def _update_ports(self):
        """Met à jour la liste des ports série"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_combo.get():
            self.port_combo.current(0)
    
    def _toggle_connection(self):
        """Connecte/Déconnecte du port série"""
        if self.running:
            self._disconnect()
        else:
            self._connect()
    
    def _connect(self):
        """Connecte au port série"""
        port = self.port_combo.get()
        if not port:
            self._log_console("❌ Aucun port sélectionné")
            return
        
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            self.running = True
            
            # Démarrer le thread de lecture
            self.read_thread = threading.Thread(target=self._read_serial, daemon=True)
            self.read_thread.start()
            
            self.connect_btn.config(text="Déconnecter")
            self.status_label.config(text="✅ Connecté", foreground="green")
            self._log_console(f"✅ Connecté à {port}\n")
            
        except Exception as e:
            self._log_console(f"❌ Erreur de connexion: {e}\n")
    
    def _disconnect(self):
        """Déconnecte du port série"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        
        self.connect_btn.config(text="Connecter")
        self.status_label.config(text="❌ Déconnecté", foreground="red")
        self._log_console("❌ Déconnecté\n")
    
    def _read_serial(self):
        """Thread de lecture du port série"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        # Afficher dans la console
                        self._log_console(line)
                        
                        # Essayer de parser le JSON
                        if line.startswith('{'):
                            try:
                                data = json.loads(line)
                                self.data_queue.put(data)
                                # DEBUG: afficher ce qu'on a parsé
                                if 'beacons' in data:
                                    self._log_console("[DEBUG] Beacons reçus: %d" % len(data['beacons']))
                            except json.JSONDecodeError as e:
                                self._log_console("[WARN] JSON invalide: %s" % str(e))
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.running:
                    self._log_console(" Erreur lecture: %s" % str(e))
                    time.sleep(0.1)
    
    def _update(self):
        """Met à jour l'interface périodiquement"""
        # Traiter les données de la queue
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.latest_data = data
                self._update_displays(data)
            except queue.Empty:
                break
        
        # Relancer après 50ms
        self.root.after(50, self._update)
    
    def _update_displays(self, data):
        """Met à jour tous les affichages"""
        
        # DEBUG: Afficher les clés reçues
        # self._log_console("[DEBUG] Clés JSON: %s" % str(list(data.keys())))
        
        # Boussole
        if 'compass' in data:
            heading = data['compass'].get('heading', 0)
            cardinal = data['compass'].get('cardinal', 'N')
            
            self.heading_label.config(text="Cap: %.1f°" % heading)
            self.cardinal_label.config(text="Direction: %s" % cardinal)
            
            self._draw_compass(heading)
            
            # Magnétomètre (optionnel, peut ne pas être présent)
            if 'mag_x' in data['compass']:
                mag_x = data['compass'].get('mag_x', 0)
                mag_y = data['compass'].get('mag_y', 0)
                mag_z = data['compass'].get('mag_z', 0)
                self.mag_label.config(text="Magnétomètre: X=%7.2f Y=%7.2f Z=%7.2f µT" % (mag_x, mag_y, mag_z))
            else:
                self.mag_label.config(text="Magnétomètre: N/A")
        
        # Accéléromètre (optionnel, peut ne pas être présent dans le nouveau code)
        if 'accel' in data:
            accel_x = data['accel'].get('x', 0)
            accel_y = data['accel'].get('y', 0)
            accel_z = data['accel'].get('z', 0)
            self.accel_label.config(text="Accéléromètre: X=%7.3f Y=%7.3f Z=%7.3f g" % (accel_x, accel_y, accel_z))
        else:
            self.accel_label.config(text="Accéléromètre: N/A")
        
        # BLE - Nouveau format avec liste de beacons
        if 'beacons' in data:
            beacons_list = data['beacons']
            
            # DEBUG
            # self._log_console("[DEBUG] Traitement de %d beacons" % len(beacons_list))
            
            # Vider le treeview
            for item in self.ble_tree.get_children():
                self.ble_tree.delete(item)
            
            # Filtrer et ajouter SEULEMENT les beacons autorisés
            filtered_count = 0
            for beacon in beacons_list:
                addr = beacon.get('addr', 'N/A')
                
                #  FILTRAGE: Ignorer les beacons non autorisés
                if addr not in ALLOWED_MACS:
                    continue
                
                # Utiliser le nom personnalisé si disponible
                name = BEACON_NAMES.get(addr, beacon.get('name', 'Unknown'))
                rssi = beacon.get('rssi_filtered', beacon.get('rssi', 0))
                distance = beacon.get('distance', 0)
                
                # Couleur selon le beacon
                if addr == "1C:DB:D4:34:74:DE":  # Bombe
                    tag = "target"
                else:
                    tag = "fixed"
                
                # Afficher
                item_id = self.ble_tree.insert('', tk.END, 
                                               values=(name, addr, "%.1f" % rssi, "%.2fm" % distance),
                                               tags=(tag,))
                filtered_count += 1
            
            # Configurer les couleurs
            self.ble_tree.tag_configure("target", background="#ffcccc")  # Rouge clair pour la Bombe
            self.ble_tree.tag_configure("fixed", background="#ccffcc")   # Vert clair pour les balises fixes
            
            self.ble_count_label.config(text="%d/%d beacons autorisés détectés" % (filtered_count, len(ALLOWED_MACS)))
        
        # Ancien format BLE (pour compatibilité)
        elif 'ble' in data:
            devices = data['ble'].get('devices', [])
            count = data['ble'].get('devices_count', 0)
            
            # Vider le treeview
            for item in self.ble_tree.get_children():
                self.ble_tree.delete(item)
            
            # Filtrer et ajouter SEULEMENT les beacons autorisés
            filtered_count = 0
            for device in devices:
                addr = device.get('addr', 'N/A')
                
                #  FILTRAGE: Ignorer les devices non autorisés
                if addr not in ALLOWED_MACS:
                    continue
                
                # Utiliser le nom personnalisé
                name = BEACON_NAMES.get(addr, device.get('name', 'Unknown'))
                rssi = device.get('rssi', 0)
                
                # Couleur selon le beacon
                if addr == "1C:DB:D4:34:74:DE":  # Bombe
                    tag = "target"
                else:
                    tag = "fixed"
                
                item_id = self.ble_tree.insert('', tk.END, 
                                               values=(name, addr, "%.1f" % rssi, "N/A"),
                                               tags=(tag,))
                filtered_count += 1
            
            # Configurer les couleurs
            self.ble_tree.tag_configure("target", background="#ffcccc")
            self.ble_tree.tag_configure("fixed", background="#ccffcc")
            
            self.ble_count_label.config(text="%d/%d beacons autorisés détectés" % (filtered_count, len(ALLOWED_MACS)))
        
        # Position de la cible (NOUVEAU)
        if 'target_position_2d' in data and data['target_position_2d']:
            pos = data['target_position_2d']
            if pos.get('x') is not None and pos.get('y') is not None:
                self._log_console("[POSITION 2D] x=%.2fm, y=%.2fm" % (pos['x'], pos['y']))
        
        if 'target_position_3d' in data and data['target_position_3d']:
            pos = data['target_position_3d']
            if pos.get('x') is not None and pos.get('y') is not None and pos.get('z') is not None:
                self._log_console("[POSITION 3D] x=%.2fm, y=%.2fm, z=%.2fm" % (pos['x'], pos['y'], pos['z']))
    
    def _draw_compass(self, heading):
        """Dessine la boussole"""
        canvas = self.compass_canvas
        canvas.delete("all")
        
        # Dimensions
        cx, cy = 150, 150
        radius = 120
        
        # Cercle extérieur
        canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline="black", width=2)
        
        # Points cardinaux
        points = [("N", 0), ("E", 90), ("S", 180), ("O", 270)]
        for label, angle in points:
            rad = math.radians(angle)
            x = cx + (radius + 20) * math.sin(rad)
            y = cy - (radius + 20) * math.cos(rad)
            canvas.create_text(x, y, text=label, font=("Arial", 14, "bold"))
        
        # Aiguille (rouge pour le nord)
        heading_rad = math.radians(heading)
        x_tip = cx + (radius - 20) * math.sin(heading_rad)
        y_tip = cy - (radius - 20) * math.cos(heading_rad)
        
        # Queue de l'aiguille
        x_tail = cx - 15 * math.sin(heading_rad)
        y_tail = cy + 15 * math.cos(heading_rad)
        
        # Dessiner l'aiguille
        canvas.create_line(cx, cy, x_tip, y_tip, fill="red", width=3, arrow=tk.LAST, arrowshape=(16,20,6))
        canvas.create_line(cx, cy, x_tail, y_tail, fill="gray", width=2)
        
        # Point central
        canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="black")


def main():
    root = tk.Tk()
    app = CompassBLEInterface(root)
    root.mainloop()


if __name__ == "__main__":
    main()